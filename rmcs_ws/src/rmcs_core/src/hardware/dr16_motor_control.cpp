#include <rclcpp/node.hpp>
// ROS2 的 Node 定义。
// 我们后面要从 YAML / ROS2 参数中读取电机和 PID 配置。

#include <rmcs_executor/component.hpp>
// RMCS 的 Component 基类。
// 只有继承它，当前类才能进入 RMCS 的组件执行系统。

namespace rmcs_core::hardware {

// 这是我们这次任务新建的 hardware 类。
// 当前第一版只是“最小骨架”，暂时还没有接电机和 DR16。
class Dr16MotorControl
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Dr16MotorControl()
        : Node{
              get_component_name(),
              // 使用 RMCS Component 的名称作为 ROS2 Node 名称。

              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)
              // 允许以后直接从 YAML 配置中读取传进来的参数。
          } {
        // 当前阶段故意保持空构造函数。
        // 下一阶段再逐步加入 Board、DR16 和 DjiMotor。
    }

    void update() override {
        // RMCS executor 会周期调用 update()。
        //
        // 当前第一阶段什么都不做。
        // 后面这里会逐步加入：
        // 1. 更新电机反馈
        // 2. 更新 DR16
        // 3. 读取摇杆
        // 4. 速度滤波
        // 5. PID 计算
    }
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
// pluginlib 的插件导出宏定义。

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::Dr16MotorControl,
    rmcs_executor::Component
)
// 把 Dr16MotorControl 导出成 rmcs_executor::Component 插件。
// 如果没有这一句，即使代码编译进共享库，pluginlib 也无法正常创建这个类。