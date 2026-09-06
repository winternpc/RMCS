from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path

import yaml


TOOL_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(TOOL_ROOT))

from tuning_core import (  # noqa: E402
    Candidate,
    Sample,
    bounded_nelder_mead,
    calculate_metrics,
    load_and_validate_template,
    temporary_candidate_yaml,
)


TEMPLATE = (
    REPO_ROOT / "rmcs_ws/src/rmcs_bringup/config/dr16-motor-autotune.yaml"
)


class MetricsTests(unittest.TestCase):
    @staticmethod
    def make_response(good: bool) -> list[Sample]:
        samples = []
        for index in range(51):
            time_value = index * 0.1
            velocity = min(5.0, time_value * 10.0) if good else min(3.0, time_value * 4.0)
            samples.append(
                Sample(
                    time=time_value,
                    step_index=1,
                    target_velocity=5.0,
                    velocity=velocity,
                    control_torque=0.1 if good else 0.2,
                )
            )
        for index in range(31):
            time_value = 5.1 + index * 0.1
            velocity = max(0.0, 5.0 - index) if good else max(0.0, 3.0 - index * 0.1)
            samples.append(
                Sample(
                    time=time_value,
                    step_index=2,
                    target_velocity=0.0,
                    velocity=velocity,
                    control_torque=-0.1 if good else -0.2,
                )
            )
        return samples

    def test_good_response_scores_better_than_slow_saturated_response(self) -> None:
        candidate = Candidate(0.05, 0.0005)
        good = calculate_metrics(candidate, self.make_response(True))
        bad = calculate_metrics(candidate, self.make_response(False))
        self.assertLess(good.score, bad.score)
        self.assertFalse(good.timeout)
        self.assertTrue(bad.timeout)
        self.assertEqual(good.saturation_time, 0.0)
        self.assertGreater(bad.saturation_time, 7.0)

    def test_incomplete_sequence_gets_failure_score(self) -> None:
        metrics = calculate_metrics(
            Candidate(0.05, 0.0005),
            [Sample(0.0, 1, 5.0, 0.0, 0.2)],
        )
        self.assertTrue(metrics.timeout)
        self.assertTrue(math.isinf(metrics.iae))
        self.assertEqual(metrics.score, 1_000_000.0)


class OptimizerTests(unittest.TestCase):
    def test_bounded_nelder_mead_converges_without_a_fixed_grid(self) -> None:
        result = bounded_nelder_mead(
            lambda point: (point[0] - 0.123) ** 2
            + 1_000_000.0 * (point[1] - 0.00073) ** 2,
            initial=(0.05, 0.0005),
            initial_steps=(0.04, 0.0003),
            bounds=((0.0, 0.3), (0.0, 0.0015)),
            max_evaluations=60,
            x_tolerance=(0.0001, 0.000001),
        )
        self.assertAlmostEqual(result.best_point[0], 0.123, delta=0.005)
        self.assertAlmostEqual(result.best_point[1], 0.00073, delta=0.00002)
        self.assertGreater(result.evaluations, 3)
        self.assertTrue(all(0.0 <= point[0] <= 0.3 for point, _ in result.history))
        self.assertTrue(all(0.0 <= point[1] <= 0.0015 for point, _ in result.history))

    def test_optimizer_rejects_too_few_evaluations(self) -> None:
        with self.assertRaises(ValueError):
            bounded_nelder_mead(
                lambda point: sum(point),
                initial=(0.05, 0.0005),
                initial_steps=(0.01, 0.0001),
                bounds=((0.0, 0.3), (0.0, 0.0015)),
                max_evaluations=2,
            )


class YamlTests(unittest.TestCase):
    def test_candidate_yaml_changes_only_kp_and_ki(self) -> None:
        original = TEMPLATE.read_bytes()
        candidate = Candidate(0.071234, 0.000321)
        with temporary_candidate_yaml(TEMPLATE, candidate) as generated:
            data = load_and_validate_template(generated)
            pid = data["velocity_pid_controller"]["ros__parameters"]
            self.assertEqual(pid["kp"], candidate.kp)
            self.assertEqual(pid["ki"], candidate.ki)
            self.assertEqual(pid["kd"], 0.0)
            self.assertEqual(pid["output_min"], -0.2)
            self.assertEqual(pid["output_max"], 0.2)
        self.assertEqual(TEMPLATE.read_bytes(), original)

    def test_torque_limit_change_is_rejected(self) -> None:
        data = load_and_validate_template(TEMPLATE)
        data["velocity_pid_controller"]["ros__parameters"]["output_max"] = 0.3
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "unsafe.yaml"
            with path.open("w", encoding="utf-8") as stream:
                yaml.safe_dump(data, stream, sort_keys=False)
            with self.assertRaisesRegex(ValueError, "torque limit"):
                load_and_validate_template(path)


if __name__ == "__main__":
    unittest.main()
