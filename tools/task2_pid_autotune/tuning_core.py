"""Pure-Python metrics, YAML generation, and bounded Nelder-Mead search."""

from __future__ import annotations

import math
import tempfile
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, Iterator, List, Sequence, Tuple

import yaml


TORQUE_LIMIT = 0.2
KP_BOUNDS = (0.0, 0.3)
KI_BOUNDS = (0.0, 0.0015)
AUTOTUNE_TARGET_TOPIC = "/motor/autotune_target_velocity"
OBSERVED_TOPICS = {
    "target_velocity": "/motor/target_velocity",
    "velocity": "/motor/velocity_filtered",
    "control_torque": "/motor/control_torque",
}


@dataclass(frozen=True)
class Candidate:
    kp: float
    ki: float

    @property
    def candidate_id(self) -> str:
        return f"kp_{self.kp:.8g}_ki_{self.ki:.8g}"


@dataclass(frozen=True)
class Sample:
    time: float
    step_index: int
    target_velocity: float
    velocity: float
    control_torque: float


@dataclass(frozen=True)
class TestConfig:
    target_velocity: float = 5.0
    initial_hold: float = 1.0
    driven_hold: float = 5.0
    return_hold: float = 3.0
    sample_period: float = 0.01
    target_publish_period: float = 0.04
    topic_timeout: float = 0.3
    startup_timeout: float = 10.0
    stable_error: float = 0.25
    stable_duration: float = 0.5
    final_window: float = 1.0
    torque_saturation_threshold: float = 0.198
    torque_violation_epsilon: float = 0.001
    initial_velocity_limit: float = 0.2
    initial_quiet_duration: float = 0.5
    safe_velocity_limit: float = 20.0


TEST_CONFIG = TestConfig()


@dataclass
class CandidateMetrics:
    candidate_id: str
    kp: float
    ki: float
    score: float
    steady_state_error: float
    iae: float
    overshoot: float
    settling_time: float | None
    return_settling_time: float | None
    saturation_time: float
    max_torque: float
    max_velocity: float
    timeout: bool
    safety_abort: bool
    stop_reason: str


@dataclass(frozen=True)
class OptimizationResult:
    best_point: Tuple[float, float]
    best_value: float
    evaluations: int
    history: Tuple[Tuple[Tuple[float, float], float], ...]


def _duration_where(samples: Sequence[Sample], predicate: Callable[[Sample], bool]) -> float:
    total = 0.0
    for current, following in zip(samples, samples[1:]):
        if predicate(current):
            total += max(0.0, following.time - current.time)
    return total


def _iae(samples: Sequence[Sample]) -> float:
    total = 0.0
    for current, following in zip(samples, samples[1:]):
        dt = max(0.0, following.time - current.time)
        current_error = abs(current.target_velocity - current.velocity)
        following_error = abs(following.target_velocity - following.velocity)
        total += 0.5 * (current_error + following_error) * dt
    return total


def _settling_time(
    samples: Sequence[Sample], config: TestConfig
) -> float | None:
    if len(samples) < 2:
        return None
    start = samples[0].time
    end = samples[-1].time
    for index, sample in enumerate(samples):
        remaining = samples[index:]
        if end - sample.time < config.stable_duration:
            break
        if all(
            abs(item.target_velocity - item.velocity) <= config.stable_error
            for item in remaining
        ):
            return sample.time - start
    return None


def calculate_metrics(
    candidate: Candidate,
    samples: Sequence[Sample],
    *,
    safety_abort: bool = False,
    stop_reason: str = "",
    config: TestConfig = TEST_CONFIG,
) -> CandidateMetrics:
    ordered = sorted(samples, key=lambda sample: sample.time)
    driven = [sample for sample in ordered if sample.step_index == 1]
    returned = [sample for sample in ordered if sample.step_index == 2]
    if not driven or not returned:
        return CandidateMetrics(
            candidate_id=candidate.candidate_id,
            kp=candidate.kp,
            ki=candidate.ki,
            score=1_000_000.0,
            steady_state_error=math.inf,
            iae=math.inf,
            overshoot=math.inf,
            settling_time=None,
            return_settling_time=None,
            saturation_time=0.0,
            max_torque=math.inf,
            max_velocity=math.inf,
            timeout=True,
            safety_abort=safety_abort,
            stop_reason=stop_reason or "incomplete 0 -> step -> 0 sequence",
        )

    final_time = driven[-1].time
    final_window = [
        sample for sample in driven if sample.time >= final_time - config.final_window
    ]
    steady_state_error = sum(
        abs(sample.target_velocity - sample.velocity) for sample in final_window
    ) / len(final_window)

    direction = 1.0 if config.target_velocity >= 0.0 else -1.0
    driven_overshoot = max(
        0.0,
        max(direction * (sample.velocity - sample.target_velocity) for sample in driven),
    )
    return_overshoot = max(0.0, max(-direction * sample.velocity for sample in returned))
    overshoot = max(driven_overshoot, return_overshoot)

    settling_time = _settling_time(driven, config)
    return_settling_time = _settling_time(returned, config)
    iae = _iae(driven) + _iae(returned)
    evaluated = driven + returned
    saturation_time = _duration_where(
        driven,
        lambda sample: abs(sample.control_torque)
        >= config.torque_saturation_threshold,
    ) + _duration_where(
        returned,
        lambda sample: abs(sample.control_torque)
        >= config.torque_saturation_threshold,
    )
    timeout = settling_time is None or return_settling_time is None
    settling_penalty = (
        settling_time if settling_time is not None else config.driven_hold
    ) + (
        return_settling_time if return_settling_time is not None else config.return_hold
    )
    score = (
        20.0 * steady_state_error
        + 2.0 * iae
        + 10.0 * overshoot
        + 2.0 * settling_penalty
        + saturation_time
        + (50.0 if timeout else 0.0)
        + (100_000.0 if safety_abort else 0.0)
    )

    return CandidateMetrics(
        candidate_id=candidate.candidate_id,
        kp=candidate.kp,
        ki=candidate.ki,
        score=score,
        steady_state_error=steady_state_error,
        iae=iae,
        overshoot=overshoot,
        settling_time=settling_time,
        return_settling_time=return_settling_time,
        saturation_time=saturation_time,
        max_torque=max(abs(sample.control_torque) for sample in evaluated),
        max_velocity=max(abs(sample.velocity) for sample in evaluated),
        timeout=timeout,
        safety_abort=safety_abort,
        stop_reason=stop_reason,
    )


def validate_candidate(
    candidate: Candidate,
    bounds: Sequence[Tuple[float, float]] = (KP_BOUNDS, KI_BOUNDS),
) -> None:
    if not all(math.isfinite(value) for value in (candidate.kp, candidate.ki)):
        raise ValueError("PID gains must be finite")
    for name, value, (lower, upper) in zip(
        ("kp", "ki"), (candidate.kp, candidate.ki), bounds
    ):
        if not lower <= value <= upper:
            raise ValueError(f"{name}={value} is outside [{lower}, {upper}]")


def load_and_validate_template(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    try:
        executor = data["rmcs_executor"]["ros__parameters"]
        hardware = data["dr16_motor_control"]["ros__parameters"]
        pid = data["velocity_pid_controller"]["ros__parameters"]
        forwarded = data["value_broadcaster"]["ros__parameters"]["forward_list"]
    except (KeyError, TypeError) as error:
        raise ValueError(f"unexpected autotune YAML structure: {error}") from error

    if executor.get("update_rate") != 1000.0:
        raise ValueError("autotune update_rate must remain 1000 Hz")
    if hardware.get("autotune_mode") is not True:
        raise ValueError("autotune_mode must be true in the autotune template")
    if pid.get("measurement") != OBSERVED_TOPICS["velocity"]:
        raise ValueError("unexpected PID measurement topic")
    if pid.get("setpoint") != OBSERVED_TOPICS["target_velocity"]:
        raise ValueError("unexpected PID setpoint topic")
    if pid.get("control") != OBSERVED_TOPICS["control_torque"]:
        raise ValueError("unexpected PID control topic")
    if pid.get("kd") != 0.0:
        raise ValueError("autotune only supports PI; kd must remain zero")
    if pid.get("output_min") != -TORQUE_LIMIT or pid.get("output_max") != TORQUE_LIMIT:
        raise ValueError("torque limit must remain exactly +/-0.2")
    missing = set(OBSERVED_TOPICS.values()).difference(forwarded)
    if missing:
        raise ValueError(f"ValueBroadcaster is missing topics: {sorted(missing)}")
    return data


def write_candidate_yaml(
    template: Path,
    destination: Path,
    candidate: Candidate,
    bounds: Sequence[Tuple[float, float]] = (KP_BOUNDS, KI_BOUNDS),
) -> None:
    validate_candidate(candidate, bounds)
    data = load_and_validate_template(template)
    pid = data["velocity_pid_controller"]["ros__parameters"]
    pid["kp"] = candidate.kp
    pid["ki"] = candidate.ki
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)


@contextmanager
def temporary_candidate_yaml(
    template: Path,
    candidate: Candidate,
    bounds: Sequence[Tuple[float, float]] = (KP_BOUNDS, KI_BOUNDS),
) -> Iterator[Path]:
    with tempfile.TemporaryDirectory(prefix="task2_pid_autotune_") as directory:
        path = Path(directory) / "dr16-motor-autotune.yaml"
        write_candidate_yaml(template, path, candidate, bounds)
        yield path


def _project(
    point: Sequence[float], bounds: Sequence[Tuple[float, float]]
) -> Tuple[float, ...]:
    return tuple(
        min(upper, max(lower, float(value)))
        for value, (lower, upper) in zip(point, bounds)
    )


def bounded_nelder_mead(
    objective: Callable[[Tuple[float, float]], float],
    initial: Tuple[float, float],
    initial_steps: Tuple[float, float],
    bounds: Sequence[Tuple[float, float]],
    *,
    max_evaluations: int = 15,
    x_tolerance: Tuple[float, float] = (0.0025, 0.000025),
) -> OptimizationResult:
    """Run a two-dimensional, bound-projected Nelder-Mead search."""

    if len(bounds) != 2 or len(initial_steps) != 2:
        raise ValueError("this autotuner requires exactly kp and ki")
    if max_evaluations < 3:
        raise ValueError("Nelder-Mead requires at least three evaluations")
    if any(step <= 0.0 for step in initial_steps):
        raise ValueError("initial simplex steps must be positive")

    cache: Dict[Tuple[float, float], float] = {}
    history: List[Tuple[Tuple[float, float], float]] = []

    def evaluate(raw_point: Sequence[float]) -> float:
        point = _project(raw_point, bounds)
        key = (round(point[0], 12), round(point[1], 12))
        if key in cache:
            return cache[key]
        if len(history) >= max_evaluations:
            raise StopIteration
        value = float(objective((point[0], point[1])))
        if not math.isfinite(value):
            value = 1_000_000.0
        cache[key] = value
        history.append(((point[0], point[1]), value))
        return value

    start = _project(initial, bounds)
    simplex: List[Tuple[float, float]] = [start]
    for axis, step in enumerate(initial_steps):
        trial = list(start)
        trial[axis] += step
        projected = _project(trial, bounds)
        if projected == start:
            trial[axis] = start[axis] - step
            projected = _project(trial, bounds)
        if projected == start:
            raise ValueError("initial simplex collapsed at a parameter bound")
        simplex.append((projected[0], projected[1]))

    values = [evaluate(point) for point in simplex]
    alpha, gamma, rho, sigma = 1.0, 2.0, 0.5, 0.5

    try:
        while len(history) < max_evaluations:
            ranked = sorted(zip(values, simplex), key=lambda item: item[0])
            values = [item[0] for item in ranked]
            simplex = [item[1] for item in ranked]

            if all(
                max(point[axis] for point in simplex)
                - min(point[axis] for point in simplex)
                <= x_tolerance[axis]
                for axis in range(2)
            ):
                break

            centroid = tuple(
                sum(point[axis] for point in simplex[:-1]) / 2.0
                for axis in range(2)
            )
            worst = simplex[-1]
            reflected = _project(
                [centroid[i] + alpha * (centroid[i] - worst[i]) for i in range(2)],
                bounds,
            )
            reflected_value = evaluate(reflected)

            if values[0] <= reflected_value < values[-2]:
                simplex[-1], values[-1] = reflected, reflected_value
                continue

            if reflected_value < values[0]:
                expanded = _project(
                    [
                        centroid[i] + gamma * (reflected[i] - centroid[i])
                        for i in range(2)
                    ],
                    bounds,
                )
                expanded_value = evaluate(expanded)
                if expanded_value < reflected_value:
                    simplex[-1], values[-1] = expanded, expanded_value
                else:
                    simplex[-1], values[-1] = reflected, reflected_value
                continue

            contracted = _project(
                [centroid[i] + rho * (worst[i] - centroid[i]) for i in range(2)],
                bounds,
            )
            contracted_value = evaluate(contracted)
            if contracted_value < values[-1]:
                simplex[-1], values[-1] = contracted, contracted_value
                continue

            best = simplex[0]
            for index in range(1, 3):
                simplex[index] = _project(
                    [
                        best[axis] + sigma * (simplex[index][axis] - best[axis])
                        for axis in range(2)
                    ],
                    bounds,
                )
                values[index] = evaluate(simplex[index])
    except StopIteration:
        pass

    best_point, best_value = min(history, key=lambda item: item[1])
    return OptimizationResult(
        best_point=best_point,
        best_value=best_value,
        evaluations=len(history),
        history=tuple(history),
    )
