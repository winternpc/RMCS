# RMCS 第二周任务代码说明

这个 README 用来整理第二周任务涉及的代码位置、各文件之间的关系，以及验收时建议优先查看的部分。

本周主要完成了三部分内容：

1. 阅读普通步兵 17 mm 发射机构代码并整理组件串联关系；
2. 新增 DR16 控制 M3508 的速度闭环，加入速度反馈低通滤波和 PID；
3. 在速度环外增加角度环，并实现按优弧（长弧）到达目标角度。

另外，为了减少手动反复调 PID 的工作量，又单独做了一个任务二 AutoTune 工具，用来自动执行阶跃测试和搜索 PI 参数。

当前最终验收代码已经整理到 `main`：

```text
main
├── WEEK2_README.md
├── 任务二正式速度闭环与最终 PI 参数
└── 任务三角度双环与优弧修正版

experiment/task2-velocity-pid-autotune
└── AutoTune 实验工具
```

因此只看本周最终任务代码时直接看 `main` 即可；想继续看自动调参实现时再切到实验分支。

---

## 1. 代码结构

和本周任务直接相关的目录大致如下：

```text
RMCS/
├── rmcs_ws/
│   └── src/
│       ├── rmcs_core/
│       │   └── src/
│       │       ├── hardware/
│       │       │   ├── dr16_motor_control.cpp
│       │       │   └── angle_motor_hardware.cpp
│       │       │
│       │       └── controller/
│       │           └── angle_target_controller.cpp
│       │
│       └── rmcs_bringup/
│           └── config/
│               ├── dr16-motor-control.yaml
│               ├── dr16-motor-autotune.yaml
│               └── angle-motor-control.yaml
│
└── tools/
    └── task2_pid_autotune/
        ├── task2_pid_autotune.py
        ├── tuning_core.py
        ├── README.md
        └── tests/
            └── test_tuning_core.py
```

如果只想快速看本周任务代码，建议先看下面这些文件。

---

# 2. 任务一：发射机构代码阅读

任务一没有重新写一套发射机构，而是阅读现有普通步兵代码并整理组件关系。

主要阅读：

- [`rmcs_ws/src/rmcs_bringup/config/omni-infantry.yaml`](rmcs_ws/src/rmcs_bringup/config/omni-infantry.yaml)
- `friction_wheel_controller.cpp`
- `heat_controller.cpp`
- `bullet_feeder_controller_17mm.cpp`
- [`rmcs_ws/src/rmcs_core/src/hardware/omni_infantry.cpp`](rmcs_ws/src/rmcs_core/src/hardware/omni_infantry.cpp)

最后整理出的主要关系可以概括为：

```text
遥控器 / 裁判系统
        ↓
摩擦轮、热量、拨弹逻辑
        ↓
三个速度 PID
        ↓
Hardware
        ↓
CAN
        ↓
摩擦轮电机 / 拨弹轮电机
```

这一部分主要用来理解 RMCS 中：

```text
YAML
→ Controller
→ PID
→ Command / Partner
→ Hardware
→ Motor
```

是怎样串起来的，任务二和任务三基本都沿用了这个思路。

---

# 3. 任务二：DR16 电机速度闭环

## 3.1 核心 Hardware

### [`dr16_motor_control.cpp`](rmcs_ws/src/rmcs_core/src/hardware/dr16_motor_control.cpp)

这是任务二最核心的文件。

主要完成：

```text
DR16 串口状态更新
→ 读取左摇杆 Y 轴
→ 映射成目标速度
→ 更新 M3508 反馈
→ 速度反馈低通滤波
→ 检查 DR16 / 电机反馈是否有效
→ 接收 PID 控制扭矩
→ 生成电机命令并通过 CAN 下发
```

目标速度映射关系为：

```text
target_velocity = joystick_y × max_target_velocity
```

本次：

```text
max_target_velocity = 10 rad/s
```

所以摇杆范围大致对应：

```text
最上方   → +10 rad/s
中位     → 0 rad/s
最下方   → -10 rad/s
```

### 主要参考的现有代码

任务二没有从空白重新实现底层协议，主要参考了：

- [`flight.cpp`](rmcs_ws/src/rmcs_core/src/hardware/flight.cpp)：DR16 数据和 Hardware 的组织方式；
- [`dr16.hpp`](rmcs_ws/src/rmcs_core/src/hardware/device/dr16.hpp)：直接使用已经解析、归一化后的摇杆值；
- [`dji_motor.hpp`](rmcs_ws/src/rmcs_core/src/hardware/device/dji_motor.hpp)：M3508 速度反馈和控制命令；
- `deformable_suspension.cpp`：`LowPassFilter<1>` 的实际用法；
- [`omni_infantry.cpp`](rmcs_ws/src/rmcs_core/src/hardware/omni_infantry.cpp)：Command / Partner 和 Hardware 的组织方式。

---

## 3.2 任务二配置

### [`dr16-motor-control.yaml`](rmcs_ws/src/rmcs_bringup/config/dr16-motor-control.yaml)

这个文件负责把任务二的各组件真正串起来。

主要关系：

```text
DR16
  ↓
/motor/target_velocity
  ↓
PidController
  ↑
/motor/velocity_filtered
  ↓
/motor/control_torque
  ↓
Dr16MotorControl Hardware
  ↓
M3508
```

最终正式使用的 PI 参数：

```yaml
kp: 0.131875
ki: 0.00078125
kd: 0.0
```

速度反馈使用一阶低通滤波，截止频率为：

```text
10 Hz
```

控制循环频率约：

```text
1000 Hz
```

---

# 4. 任务二：PID AutoTune

AutoTune 不是题目要求必须实现的主体功能，是在任务二已经能够手动闭环以后，为了减少重复调参工作额外做的工具。

## 4.1 为什么做 AutoTune

最开始 PID 是手动调的：

```text
修改 Kp / Ki
→ 启动程序
→ 推摇杆
→ 看 target / filtered
→ 判断误差和超调
→ 再修改参数
```

实际做起来以后发现，这个过程非常重复，而且某组参数在 5 rad/s 合适，不一定在 8 或 10 rad/s 也一样。

所以后来把固定的测试流程交给程序：

```text
自动给定目标速度
→ 执行阶跃
→ 记录速度和控制量
→ 计算稳态误差 / 超调 / 调节时间等指标
→ 尝试下一组 Kp、Ki
```

这部分代码是在测试思路先确定以后，再使用 Codex 辅助完成实现和迭代。

---

## 4.2 主要文件

### [`tools/task2_pid_autotune/task2_pid_autotune.py`](https://github.com/winternpc/RMCS/blob/experiment/task2-velocity-pid-autotune/tools/task2_pid_autotune/task2_pid_autotune.py)

主要负责 ROS2 和实机测试流程：

```text
发布目标速度
订阅 target / filtered / control_torque
执行 0 → target → 0 阶跃
记录每轮实验数据
保存 CSV
```

### [`tools/task2_pid_autotune/tuning_core.py`](https://github.com/winternpc/RMCS/blob/experiment/task2-velocity-pid-autotune/tools/task2_pid_autotune/tuning_core.py)

主要负责：

```text
候选 PI 参数管理
参数搜索
稳态误差计算
超调量计算
调节时间计算
整段误差计算
结果排序
```

搜索使用有边界的连续参数搜索，而不是只测试固定整数步长，所以最终会得到：

```text
Kp = 0.131875
Ki = 0.00078125
```

这种连续数值。

### [`dr16-motor-autotune.yaml`](https://github.com/winternpc/RMCS/blob/experiment/task2-velocity-pid-autotune/rmcs_ws/src/rmcs_bringup/config/dr16-motor-autotune.yaml)

调参专用配置。

它和正式 `dr16-motor-control.yaml` 分开，避免为了 AutoTune 改动正式任务配置。

---

## 4.3 5 / 8 / 10 rad/s 测试结果

三组测试得到的较优参数：

```text
5 rad/s
Kp = 0.126640625
Ki = 0.0006640625

8 rad/s
Kp = 0.131875
Ki = 0.00078125

10 rad/s
Kp = 0.142
Ki = 0.000855
```

最后选择 8 rad/s 工况下的：

```text
Kp = 0.131875
Ki = 0.00078125
Kd = 0
```

作为任务二正式配置。

---

# 5. 任务三：角度双环控制

任务三在任务二速度环外增加了角度环。

整体关系：

```text
/motor/target_angle
        ↓
AngleTargetController
        ↓
角度误差
        ↓
角度外环 PID
        ↓
/motor/control_velocity
        ↓
速度内环 PID
        ↓
/motor/control_torque
        ↓
AngleMotorHardware
        ↓
M3508
```

---

## 5.1 Angle Motor Hardware

### [`angle_motor_hardware.cpp`](rmcs_ws/src/rmcs_core/src/hardware/angle_motor_hardware.cpp)

主要负责：

```text
M3508 注册
CAN 收发
连续多圈角度反馈
速度反馈
接收 control_torque
反馈超时保护
```

这里重点是底层电机和反馈，不负责决定电机到底应该走短弧还是长弧。

多圈角度继续复用了 `DjiMotor` 已有能力，没有重新手写编码器过零累计。

---

## 5.2 优弧目标控制

### [`angle_target_controller.cpp`](rmcs_ws/src/rmcs_core/src/controller/angle_target_controller.cpp)

这是任务三最值得重点看的文件。

主要处理：

```text
收到 /motor/target_angle
→ 计算当前角度到目标角度的短弧
→ 改成另一侧长弧
→ 得到 continuous_target
→ 后续一直追踪该连续目标
```

核心思路：

```cpp
short_error =
    std::remainder(target - current_angle, 2.0 * std::numbers::pi);
```

然后选择另一侧长弧：

```text
short_error > 0
→ major_error = short_error - 2π

short_error < 0
→ major_error = short_error + 2π
```

最后：

```text
continuous_target = current_angle + major_error
```

这里不能每个控制周期都重新计算长弧，否则电机运动到一半以后可能重新改变方向。

因此只在收到新目标时计算一次连续目标，之后一直追踪它，直到下一次收到新的 `/motor/target_angle`。

---

## 5.3 双环配置

### [`angle-motor-control.yaml`](rmcs_ws/src/rmcs_bringup/config/angle-motor-control.yaml)

这里主要看几个组件怎样连接：

```text
AngleMotorHardware
AngleTargetController
角度 PID
速度 PID
ValueBroadcaster
```

任务三的双环结构主要是在这个 YAML 里完成连接。

---

# 6. 验收时建议先看什么

如果只想快速检查本周任务代码，可以按下面顺序。

## 任务二

第一优先：

```text
rmcs_ws/src/rmcs_core/src/hardware/dr16_motor_control.cpp
```

重点看：

```text
DR16
摇杆映射
M3508
LowPassFilter
安全判断
```

第二优先：

```text
rmcs_ws/src/rmcs_bringup/config/dr16-motor-control.yaml
```

重点看：

```text
target
measurement
control
PID 参数
组件连接
```

如果想看调参过程，再看：

```text
tools/task2_pid_autotune/
```

---

## 任务三

第一优先：

```text
rmcs_ws/src/rmcs_core/src/controller/angle_target_controller.cpp
```

重点看：

```text
优弧
continuous_target
```

第二优先：

```text
rmcs_ws/src/rmcs_bringup/config/angle-motor-control.yaml
```

重点看：

```text
角度外环
速度内环
```

第三优先：

```text
rmcs_ws/src/rmcs_core/src/hardware/angle_motor_hardware.cpp
```

重点看：

```text
多圈角度
M3508
CAN
控制扭矩
```

---

# 7. 关键 Git 记录

这次几个比较重要的提交：

```text
36c071c2  docs: add week2 task code overview
45e48b3b  feat: add task2 velocity PID autotuner
1ade625f  tune: update task2 velocity PI gains
79cc6160  fix: make task3 motor follow major arc
```

对应：

```text
36c071c2
→ 把第二周代码说明加入 main

45e48b3b
→ 增加任务二 AutoTune 工具和调参入口（实验分支）

1ade625f
→ 把最终选出的 8 rad/s PI 参数写回正式配置并合入 main

79cc6160
→ 修正任务三优弧方向并使用连续长弧目标，已合入 main
```

任务二 AutoTune 保留在实验分支：

```text
experiment/task2-velocity-pid-autotune
```

任务三优弧修正版已经合入：

```text
main
```

原来的 `fix/task3-major-arc` 分支仍可用于查看修复过程，但验收时直接看 `main` 即可。

---

# 8. 编译

在 `rmcs_ws` 下：

```bash
cd /workspaces/RMCS/rmcs_ws

CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 NINJAFLAGS=-j1 \
../.script/build-rmcs \
--packages-up-to rmcs_core rmcs_bringup \
--parallel-workers 1
```

因为本机容器环境内存比较有限，所以这里使用单线程构建，速度慢一点，但稳定性更好。

---

# 9. 运行

## 任务二

```bash
cd /workspaces/RMCS/rmcs_ws
source install/setup.zsh

ros2 run rmcs_executor rmcs_executor \
  --ros-args \
  --params-file install/share/rmcs_bringup/config/dr16-motor-control.yaml
```

## 任务三

```bash
cd /workspaces/RMCS/rmcs_ws
source install/setup.zsh

ros2 run rmcs_executor rmcs_executor \
  --ros-args \
  --params-file install/share/rmcs_bringup/config/angle-motor-control.yaml
```

发布目标角度：

```bash
ros2 topic pub --once \
  /motor/target_angle \
  std_msgs/msg/Float64 \
  "{data: 0.3}"
```

---

# 10. 总结

这周的代码不是重新搭一套独立框架，而是尽量按 RMCS 现有写法往里面增加功能。

任务二重点是：

```text
DR16
→ 目标速度
→ PID
→ M3508
→ 速度反馈
→ LowPassFilter
→ PID
```

任务三重点是：

```text
目标角度
→ 连续优弧目标
→ 角度外环
→ 目标速度
→ 速度内环
→ M3508
```

AutoTune 是在任务二已经能正常闭环以后额外增加的测试工具，主要用来减少反复手动修改 PI 参数和重复测试的工作量。
