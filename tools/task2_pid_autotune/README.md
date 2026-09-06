# Task 2 M3508 velocity PID autotune

This tool runs a hardware-in-the-loop, bounded Nelder-Mead search over the
Task 2 velocity-loop `Kp` and `Ki`. It does not use a fixed gain table and it
does not change the checked-in Task 2 YAML.

Each objective evaluation performs the same sequence:

1. Generate a temporary YAML containing the candidate `Kp` and `Ki`.
2. Start a fresh `rmcs_executor` process group.
3. Publish a safe zero target and wait for M3508 feedback, all three observed
   topics, and velocity below `0.2 rad/s` for `0.5 s`.
4. Run `0 -> 5 rad/s -> 0` while recording the target, filtered velocity, and
   control torque.
5. Calculate steady-state error, IAE, overshoot, settling times, and torque
   saturation time.
6. Stop the executor and verify that no `rmcs_executor` process remains before
   Nelder-Mead selects the next continuous point.

The original torque clamp remains `[-0.2, 0.2]`. Motor feedback loss, stale
topics, excessive velocity, communication errors, Ctrl+C, or an executor crash
stops the campaign. The ROS autotune target also expires after about `200 ms`;
the hardware component sends zero control when either it or motor feedback is
stale.

## Build and check

```bash
cd /workspaces/RMCS/rmcs_ws
colcon build --packages-select rmcs_core rmcs_bringup
source install/setup.zsh

cd /workspaces/RMCS
python3 -m compileall -q tools/task2_pid_autotune
python3 -m unittest discover -s tools/task2_pid_autotune/tests -v
python3 tools/task2_pid_autotune/task2_pid_autotune.py static-check
```

## Run on hardware

Keep the motor unloaded and be ready to press Ctrl+C:

```bash
cd /workspaces/RMCS
source rmcs_ws/install/setup.zsh
python3 tools/task2_pid_autotune/task2_pid_autotune.py run \
  --confirm-hardware \
  --target-velocity 5.0 \
  --max-candidates 15
```

The default search starts at the existing `Kp=0.05`, `Ki=0.0005`, with an
initial simplex size of `0.025` and `0.00025`. Bounds and simplex sizes are CLI
options, so Nelder-Mead can be narrowed without editing code.

Results are written to:

```text
tools/task2_pid_autotune/output/<UTC timestamp>/
  run.log
  optimization.csv
  best_summary.csv
  candidate_001/results.csv
  candidate_001/summary.csv
  ...
```

`optimization.csv` is flushed after every candidate. Lower scores are better.
