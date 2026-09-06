#include <cmath>
#include <limits>
#include <numbers>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/float64.hpp>

namespace rmcs_core::controller {

class AngleTargetController
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    AngleTargetController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}
                  .automatically_declare_parameters_from_overrides(true)
          } {

        // 读取 AngleMotorHardware 输出的连续多圈角度。
        register_input(
            "/motor/angle",
            motor_angle_
        );

        // 角度误差交给外环 ErrorPidController。
        register_output(
            "/motor/control_angle_error",
            angle_error_,
            nan_
        );

        // ROS2 Topic 接收新的目标角度。
        target_angle_subscription_ =
            create_subscription<std_msgs::msg::Float64>(
                "/motor/target_angle",
                rclcpp::QoS{0},
                [this](std_msgs::msg::Float64::UniquePtr&& msg) {

                    // 忽略 NaN / Inf 等非法目标。
                    if (!std::isfinite(msg->data))
                        return;

                    target_angle_ = msg->data;
                    target_received_ = true;

                    // 新目标只在下一次 update() 中重新选择一次优弧。
                    target_updated_ = true;
                }
            );
    }

    void update() override {

        // 没收到目标角度以前不启动位置控制。
        if (!target_received_) {
            *angle_error_ = nan_;
            return;
        }

        // 电机还没有有效角度反馈时，先不计算目标。
        if (!std::isfinite(*motor_angle_)) {
            *angle_error_ = nan_;
            return;
        }

        /*
         * 收到新的目标角度时，只在这里选择一次优弧。
         *
         * remainder() 先算两点之间的短弧，
         * 再取另一条弧，也就是优弧（长弧）。
         *
         * 得到长弧以后，把终点保存成连续多圈目标。
         * 后面的控制周期不再重新选弧，避免运行途中反向。
         */
        if (target_updated_) {

            const double short_error =
                std::remainder(
                    target_angle_ - *motor_angle_,
                    2.0 * std::numbers::pi
                );

            double major_arc_error = 0.0;

            // 起点和目标基本重合时，不强制转完整一圈。
            if (std::abs(short_error) > 1e-6) {

                if (short_error > 0.0) {

                    // 短弧向正方向，则优弧走负方向。
                    major_arc_error =
                        short_error
                        - 2.0 * std::numbers::pi;

                } else {

                    // 短弧向负方向，则优弧走正方向。
                    major_arc_error =
                        short_error
                        + 2.0 * std::numbers::pi;
                }
            }

            // 锁定本次运动真正要追踪的连续多圈目标。
            continuous_target_angle_ =
                *motor_angle_
                + major_arc_error;

            target_updated_ = false;
        }

        // 后续周期只追已经锁定的连续目标。
        *angle_error_ =
            continuous_target_angle_
            - *motor_angle_;
    }

private:
    static constexpr double nan_ =
        std::numeric_limits<double>::quiet_NaN();

    // AngleMotorHardware 输出的连续多圈角度。
    InputInterface<double> motor_angle_;

    // 交给外环 ErrorPidController 的角度误差。
    OutputInterface<double> angle_error_;

    // ROS2 Topic 中收到的原始目标角度。
    double target_angle_ = 0.0;

    // 根据优弧确定后的连续多圈目标。
    double continuous_target_angle_ = 0.0;

    // 是否已经收到过目标。
    bool target_received_ = false;

    // 是否需要根据新目标重新选择一次优弧。
    bool target_updated_ = false;

    rclcpp::Subscription<
        std_msgs::msg::Float64
    >::SharedPtr target_angle_subscription_;
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::AngleTargetController,
    rmcs_executor::Component
)