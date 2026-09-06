#!/usr/bin/env python3
"""Hardware-in-the-loop M3508 velocity PI autotuner for RMCS Task 2."""

from __future__ import annotations

import argparse
import csv
import logging
import math
import os
import re
import shutil
import signal
import subprocess
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence

from tuning_core import (
    AUTOTUNE_TARGET_TOPIC,
    KI_BOUNDS,
    KP_BOUNDS,
    OBSERVED_TOPICS,
    TEST_CONFIG,
    Candidate,
    CandidateMetrics,
    Sample,
    TestConfig,
    bounded_nelder_mead,
    calculate_metrics,
    load_and_validate_template,
    temporary_candidate_yaml,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
TEMPLATE_YAML = (
    REPO_ROOT
    / "rmcs_ws/src/rmcs_bringup/config/dr16-motor-autotune.yaml"
)
OUTPUT_ROOT = Path(__file__).resolve().parent / "output"
STOP_REQUESTED = threading.Event()


class HardSafetyError(RuntimeError):
    pass


@dataclass
class CandidateRun:
    metrics: CandidateMetrics
    samples: List[Sample]
    output_dir: Path


class ExecutorProcess:
    """Own one rmcs_executor and its dedicated process group."""

    def __init__(self, yaml_path: Path, logger: logging.Logger) -> None:
        self.yaml_path = yaml_path
        self.logger = logger
        self.process: Optional[subprocess.Popen[str]] = None
        self.reader_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._feedback_alive = False
        self._feedback_lost = False
        self._serious_error = ""
        self._deserializer_errors = deque()
        self._transient_warning_until = 0.0

    def start(self) -> None:
        command = [
            "ros2",
            "run",
            "rmcs_executor",
            "rmcs_executor",
            "--ros-args",
            "--params-file",
            str(self.yaml_path),
        ]
        self.logger.info("starting executor: %s", " ".join(command))
        self.process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self.reader_thread = threading.Thread(
            target=self._read_output,
            name="task2-autotune-executor-log",
            daemon=True,
        )
        self.reader_thread.start()

    def _read_output(self) -> None:
        assert self.process is not None and self.process.stdout is not None
        for raw_line in self.process.stdout:
            line = raw_line.rstrip()
            self.logger.info("executor | %s", line)
            self._classify_line(line)

    def _classify_line(self, line: str, now: Optional[float] = None) -> None:
        lowered = line.lower()
        timestamp = time.monotonic() if now is None else now
        with self._lock:
            if "feedback connected" in lowered or "feedback=alive" in lowered:
                self._feedback_alive = True
            if "feedback=lost" in lowered and self._feedback_alive:
                self._feedback_lost = True

            if "deserializer encountered an error while parsing input" in lowered:
                cutoff = timestamp - 2.0
                while self._deserializer_errors and self._deserializer_errors[0] < cutoff:
                    self._deserializer_errors.popleft()
                self._deserializer_errors.append(timestamp)
                if len(self._deserializer_errors) >= 3:
                    self._serious_error = "repeated librmcs deserializer errors"
                else:
                    self._transient_warning_until = max(
                        self._transient_warning_until, timestamp + 1.0
                    )
                    self.logger.warning(
                        "isolated deserializer error; temporarily holding zero target"
                    )
                return

            severe_source = re.search(
                r"(cboard|c_board|librmcs|serial|\bcan\b|transmit|send|write)",
                lowered,
            )
            severe_level = "[fatal]" in lowered or "[error]" in lowered
            if severe_source and severe_level:
                self._serious_error = line

    @property
    def feedback_alive(self) -> bool:
        with self._lock:
            return self._feedback_alive

    def transient_warning_active(self) -> bool:
        with self._lock:
            return time.monotonic() < self._transient_warning_until

    def health_error(self) -> Optional[str]:
        if self.process is None:
            return "executor was not started"
        return_code = self.process.poll()
        if return_code is not None:
            return f"executor exited unexpectedly with code {return_code}"
        with self._lock:
            if self._feedback_lost:
                return "M3508 feedback lost"
            if self._serious_error:
                return self._serious_error
        return None

    def stop(self) -> None:
        process = self.process
        if process is None:
            return
        if process.poll() is None:
            self.logger.info("stopping executor process group")
            self._signal_group(signal.SIGINT)
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._signal_group(signal.SIGTERM)
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self._signal_group(signal.SIGKILL)
                    process.wait(timeout=1.0)
        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.0)
        if process.stdout is not None:
            process.stdout.close()
        self.process = None

    def _signal_group(self, requested_signal: signal.Signals) -> None:
        assert self.process is not None
        try:
            os.killpg(self.process.pid, requested_signal)
        except ProcessLookupError:
            pass


def executor_process_ids() -> List[int]:
    result = subprocess.run(
        ["pgrep", "-x", "rmcs_executor"],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode == 1:
        return []
    if result.returncode != 0:
        raise RuntimeError(f"pgrep failed: {result.stderr.strip()}")
    return [int(item) for item in result.stdout.split()]


def create_topic_collector(config: TestConfig):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float64

    if not rclpy.ok():
        from rclpy.signals import SignalHandlerOptions

        rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)

    observed_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

    class TopicCollector(Node):
        def __init__(self) -> None:
            super().__init__("task2_pid_autotuner")
            self._lock = threading.Lock()
            self.latest: Dict[str, float] = {}
            self.last_seen: Dict[str, float] = {}
            self.topic_subscriptions = []
            for name, topic in OBSERVED_TOPICS.items():
                subscription = self.create_subscription(
                    Float64,
                    topic,
                    lambda message, field=name: self._store(field, message.data),
                    observed_qos,
                )
                self.topic_subscriptions.append(subscription)
            self.target_publisher = self.create_publisher(
                Float64, AUTOTUNE_TARGET_TOPIC, command_qos
            )

        def _store(self, name: str, value: float) -> None:
            now = time.monotonic()
            with self._lock:
                self.latest[name] = float(value)
                self.last_seen[name] = now

        def publish_target(self, target_velocity: float) -> None:
            message = Float64()
            message.data = float(target_velocity)
            self.target_publisher.publish(message)

        def readiness_problems(self) -> List[str]:
            problems = []
            for name, topic in OBSERVED_TOPICS.items():
                if self.count_publishers(topic) < 1:
                    problems.append(f"no publisher: {topic}")
                with self._lock:
                    if name not in self.last_seen:
                        problems.append(f"no message: {topic}")
            if self.target_publisher.get_subscription_count() < 1:
                problems.append(f"no subscriber: {AUTOTUNE_TARGET_TOPIC}")
            return problems

        def stale_topics(self, now: float) -> List[str]:
            with self._lock:
                return [
                    OBSERVED_TOPICS[name]
                    for name in OBSERVED_TOPICS
                    if name not in self.last_seen
                    or now - self.last_seen[name] > config.topic_timeout
                ]

        def snapshot(self, candidate_start: float, step_index: int) -> Optional[Sample]:
            now = time.monotonic()
            with self._lock:
                if any(name not in self.latest for name in OBSERVED_TOPICS):
                    return None
                values = dict(self.latest)
                last_seen = dict(self.last_seen)
            if any(
                now - last_seen[name] > config.topic_timeout for name in OBSERVED_TOPICS
            ):
                return None
            if not all(math.isfinite(values[name]) for name in OBSERVED_TOPICS):
                return None
            return Sample(
                time=now - candidate_start,
                step_index=step_index,
                target_velocity=values["target_velocity"],
                velocity=values["velocity"],
                control_torque=values["control_torque"],
            )

    return rclpy, TopicCollector()


def _check_sample_safety(sample: Sample, config: TestConfig) -> None:
    values = (sample.target_velocity, sample.velocity, sample.control_torque)
    if not all(math.isfinite(value) for value in values):
        raise HardSafetyError("a monitored ROS topic contains NaN or infinity")
    if abs(sample.control_torque) > 0.2 + config.torque_violation_epsilon:
        raise HardSafetyError("control torque exceeded the fixed +/-0.2 limit")
    if abs(sample.velocity) > config.safe_velocity_limit:
        raise HardSafetyError(
            f"velocity exceeded +/-{config.safe_velocity_limit:.1f} rad/s"
        )


def wait_until_ready(rclpy, node, process: ExecutorProcess, config: TestConfig) -> None:
    deadline = time.monotonic() + config.startup_timeout
    quiet_since: Optional[float] = None
    next_publish = 0.0
    last_problems: List[str] = []
    while time.monotonic() < deadline:
        if STOP_REQUESTED.is_set():
            raise HardSafetyError("user requested stop")
        error = process.health_error()
        if error:
            raise HardSafetyError(error)
        rclpy.spin_once(node, timeout_sec=0.01)
        now = time.monotonic()
        if now >= next_publish:
            node.publish_target(0.0)
            next_publish = now + config.target_publish_period
        last_problems = node.readiness_problems()
        if process.feedback_alive and not last_problems:
            sample = node.snapshot(now, 0)
            if sample is None:
                continue
            _check_sample_safety(sample, config)
            if process.transient_warning_active():
                quiet_since = None
            elif abs(sample.velocity) <= config.initial_velocity_limit:
                if quiet_since is None:
                    quiet_since = now
                if now - quiet_since >= config.initial_quiet_duration:
                    return
            else:
                quiet_since = None
    detail = "; ".join(last_problems) if last_problems else "feedback or stationary check"
    raise HardSafetyError(f"startup timeout: {detail}")


def run_phase(
    rclpy,
    node,
    process: ExecutorProcess,
    candidate_start: float,
    step_index: int,
    target_velocity: float,
    duration: float,
    config: TestConfig,
) -> List[Sample]:
    samples: List[Sample] = []
    deadline = time.monotonic() + duration
    next_publish = 0.0
    next_sample = 0.0
    recovery_started: Optional[float] = None

    while time.monotonic() < deadline:
        if STOP_REQUESTED.is_set():
            raise HardSafetyError("user requested stop")
        error = process.health_error()
        if error:
            raise HardSafetyError(error)
        rclpy.spin_once(node, timeout_sec=0.001)
        now = time.monotonic()
        stale = node.stale_topics(now)
        if stale:
            raise HardSafetyError(f"ROS topic timeout: {', '.join(stale)}")

        if process.transient_warning_active():
            if recovery_started is None:
                recovery_started = now
            node.publish_target(0.0)
            continue
        if recovery_started is not None:
            deadline += now - recovery_started
            recovery_started = None
            next_publish = 0.0

        if now >= next_publish:
            node.publish_target(target_velocity)
            next_publish = now + config.target_publish_period
        if now >= next_sample:
            sample = node.snapshot(candidate_start, step_index)
            next_sample = now + config.sample_period
            if sample is not None:
                _check_sample_safety(sample, config)
                samples.append(sample)
    return samples


def _write_samples(path: Path, candidate: Candidate, samples: Sequence[Sample]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "candidate_id",
                "kp",
                "ki",
                "time",
                "step_index",
                "target_velocity",
                "velocity_filtered",
                "control_torque",
            ],
        )
        writer.writeheader()
        for sample in samples:
            writer.writerow(
                {
                    "candidate_id": candidate.candidate_id,
                    "kp": candidate.kp,
                    "ki": candidate.ki,
                    "time": sample.time,
                    "step_index": sample.step_index,
                    "target_velocity": sample.target_velocity,
                    "velocity_filtered": sample.velocity,
                    "control_torque": sample.control_torque,
                }
            )


def _write_metrics(path: Path, metrics: CandidateMetrics) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(asdict(metrics)))
        writer.writeheader()
        writer.writerow(asdict(metrics))


def run_candidate(
    candidate: Candidate,
    output_dir: Path,
    logger: logging.Logger,
    config: TestConfig,
    bounds: Sequence[tuple[float, float]],
) -> CandidateRun:
    output_dir.mkdir(parents=True, exist_ok=False)
    samples: List[Sample] = []
    process: Optional[ExecutorProcess] = None
    node = None
    rclpy = None
    safety_abort = False
    stop_reason = ""

    with temporary_candidate_yaml(TEMPLATE_YAML, candidate, bounds) as yaml_path:
        process = ExecutorProcess(yaml_path, logger)
        try:
            process.start()
            rclpy, node = create_topic_collector(config)
            wait_until_ready(rclpy, node, process, config)
            candidate_start = time.monotonic()
            samples.extend(
                run_phase(
                    rclpy,
                    node,
                    process,
                    candidate_start,
                    0,
                    0.0,
                    config.initial_hold,
                    config,
                )
            )
            samples.extend(
                run_phase(
                    rclpy,
                    node,
                    process,
                    candidate_start,
                    1,
                    config.target_velocity,
                    config.driven_hold,
                    config,
                )
            )
            samples.extend(
                run_phase(
                    rclpy,
                    node,
                    process,
                    candidate_start,
                    2,
                    0.0,
                    config.return_hold,
                    config,
                )
            )
        except (HardSafetyError, RuntimeError) as error:
            safety_abort = True
            stop_reason = str(error)
            logger.error("candidate %s aborted: %s", candidate.candidate_id, error)
        finally:
            if node is not None and rclpy is not None:
                for _ in range(5):
                    node.publish_target(0.0)
                    rclpy.spin_once(node, timeout_sec=0.02)
                node.destroy_node()
            if process is not None:
                process.stop()

    residual = executor_process_ids()
    if residual:
        safety_abort = True
        stop_reason = f"executor cleanup failed; residual PIDs: {residual}"
        logger.error(stop_reason)

    metrics = calculate_metrics(
        candidate,
        samples,
        safety_abort=safety_abort,
        stop_reason=stop_reason,
        config=config,
    )
    _write_samples(output_dir / "results.csv", candidate, samples)
    _write_metrics(output_dir / "summary.csv", metrics)
    return CandidateRun(metrics=metrics, samples=samples, output_dir=output_dir)


def _append_campaign_summary(path: Path, metrics: CandidateMetrics) -> None:
    row = asdict(metrics)
    new_file = not path.exists()
    with path.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(row))
        if new_file:
            writer.writeheader()
        writer.writerow(row)
        stream.flush()


def setup_logger(output_dir: Path) -> logging.Logger:
    logger = logging.getLogger("task2_pid_autotune")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    file_handler = logging.FileHandler(output_dir / "run.log", encoding="utf-8")
    file_handler.setFormatter(formatter)
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    return logger


def check_hardware_dependencies() -> None:
    for command in ("ros2", "pgrep"):
        if shutil.which(command) is None:
            raise RuntimeError(f"required command not found: {command}")
    try:
        import rclpy  # noqa: F401
        from std_msgs.msg import Float64  # noqa: F401
    except ImportError as error:
        raise RuntimeError(
            "ROS Python environment is unavailable; source rmcs_ws/install/setup.zsh"
        ) from error


def run_autotune(args: argparse.Namespace) -> Path:
    if not args.confirm_hardware:
        raise RuntimeError("hardware run requires --confirm-hardware")
    check_hardware_dependencies()
    residual = executor_process_ids()
    if residual:
        raise RuntimeError(f"rmcs_executor already running: {residual}")

    bounds = ((args.kp_min, args.kp_max), (args.ki_min, args.ki_max))
    initial = Candidate(args.initial_kp, args.initial_ki)
    load_and_validate_template(TEMPLATE_YAML)
    from tuning_core import validate_candidate

    validate_candidate(initial, bounds)

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_utc")
    output_dir = OUTPUT_ROOT / timestamp
    output_dir.mkdir(parents=True, exist_ok=False)
    logger = setup_logger(output_dir)
    campaign_summary = output_dir / "optimization.csv"
    runs: List[CandidateRun] = []

    config = TestConfig(target_velocity=args.target_velocity)

    def objective(point: tuple[float, float]) -> float:
        candidate = Candidate(kp=point[0], ki=point[1])
        candidate_dir = output_dir / f"candidate_{len(runs) + 1:03d}"
        logger.info(
            "candidate %03d: kp=%.9g ki=%.9g",
            len(runs) + 1,
            candidate.kp,
            candidate.ki,
        )
        run = run_candidate(candidate, candidate_dir, logger, config, bounds)
        runs.append(run)
        _append_campaign_summary(campaign_summary, run.metrics)
        logger.info(
            "candidate result: score=%.6f steady=%.6f iae=%.6f "
            "overshoot=%.6f settling=%s return_settling=%s saturation=%.6f",
            run.metrics.score,
            run.metrics.steady_state_error,
            run.metrics.iae,
            run.metrics.overshoot,
            run.metrics.settling_time,
            run.metrics.return_settling_time,
            run.metrics.saturation_time,
        )
        if run.metrics.safety_abort:
            raise HardSafetyError(run.metrics.stop_reason)
        return run.metrics.score

    try:
        result = bounded_nelder_mead(
            objective,
            (initial.kp, initial.ki),
            (args.kp_step, args.ki_step),
            bounds,
            max_evaluations=args.max_candidates,
        )
    except HardSafetyError as error:
        logger.error("campaign stopped by hard safety: %s", error)
        raise
    finally:
        residual = executor_process_ids()
        if residual:
            logger.error("campaign ended with residual executor PIDs: %s", residual)

    best_run = min(runs, key=lambda run: run.metrics.score)
    _write_metrics(output_dir / "best_summary.csv", best_run.metrics)
    logger.info(
        "best candidate: kp=%.9g ki=%.9g score=%.6f after %d evaluations",
        result.best_point[0],
        result.best_point[1],
        result.best_value,
        result.evaluations,
    )
    print(f"Best Kp: {result.best_point[0]:.9g}")
    print(f"Best Ki: {result.best_point[1]:.9g}")
    print(f"Score: {result.best_value:.6f}")
    print(f"Results: {output_dir}")
    return output_dir


def static_check() -> None:
    original = TEMPLATE_YAML.read_bytes()
    template = load_and_validate_template(TEMPLATE_YAML)
    pid = template["velocity_pid_controller"]["ros__parameters"]
    candidate = Candidate(float(pid["kp"]), float(pid["ki"]))
    with temporary_candidate_yaml(TEMPLATE_YAML, candidate) as generated:
        generated_data = load_and_validate_template(generated)
        generated_pid = generated_data["velocity_pid_controller"]["ros__parameters"]
        if generated_pid["kp"] != candidate.kp or generated_pid["ki"] != candidate.ki:
            raise RuntimeError("temporary YAML did not preserve the candidate gains")
    if TEMPLATE_YAML.read_bytes() != original:
        raise RuntimeError("static check modified the autotune YAML template")
    print(f"YAML OK: {TEMPLATE_YAML}")
    print("Torque limit: -0.2 .. +0.2")
    print("Search: bounded Nelder-Mead over continuous Kp/Ki")
    print("Static check did not start ROS, rmcs_executor, or the motor.")


def install_signal_handlers() -> None:
    def request_stop(signum, _frame) -> None:
        STOP_REQUESTED.set()
        logging.getLogger("task2_pid_autotune").warning("received signal %s", signum)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("static-check", help="validate YAML and safety invariants")

    run = subparsers.add_parser("run", help="run hardware Nelder-Mead autotuning")
    run.add_argument("--confirm-hardware", action="store_true")
    run.add_argument("--target-velocity", type=float, default=5.0)
    run.add_argument("--max-candidates", type=int, default=15)
    run.add_argument("--initial-kp", type=float, default=0.05)
    run.add_argument("--initial-ki", type=float, default=0.0005)
    run.add_argument("--kp-step", type=float, default=0.025)
    run.add_argument("--ki-step", type=float, default=0.00025)
    run.add_argument("--kp-min", type=float, default=KP_BOUNDS[0])
    run.add_argument("--kp-max", type=float, default=KP_BOUNDS[1])
    run.add_argument("--ki-min", type=float, default=KI_BOUNDS[0])
    run.add_argument("--ki-max", type=float, default=KI_BOUNDS[1])
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        if args.command == "static-check":
            static_check()
            return 0
        install_signal_handlers()
        run_autotune(args)
        return 0
    except (RuntimeError, ValueError, HardSafetyError) as error:
        print(f"ERROR: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
