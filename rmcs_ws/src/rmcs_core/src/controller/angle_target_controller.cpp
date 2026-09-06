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

        // 参考 simple_gimbal_controller.cpp，读取角度反馈并输出误差。
        register_input("/motor/angle", motor_angle_);

        register_output(
            "/motor/control_angle_error",
            angle_error_,
            nan_
        );

        // 订阅方式参考 omni_infantry.cpp。
        target_angle_subscription_ =
            create_subscription<std_msgs::msg::Float64>(
                "/motor/target_angle",
                rclcpp::QoS{0},
                [this](std_msgs::msg::Float64::UniquePtr&& msg) {

                    target_angle_ = msg->data;
                    target_received_ = true;
                }
            );
    }

    void update() override {
        // 收到目标角度前不启动位置控制。
        if (!target_received_) {
            *angle_error_ = nan_;
            return;
        }

        // std::remainder() 将误差限制在 [-pi, pi]，得到最短弧。
        *angle_error_ =
            std::remainder(
                target_angle_ - *motor_angle_,
                2.0 * std::numbers::pi
            );
    }

private:
    static constexpr double nan_ =
        std::numeric_limits<double>::quiet_NaN();

    // AngleMotorHardware 输出的多圈角度。
    InputInterface<double> motor_angle_;

    // 交给外环 ErrorPidController 的角度误差。
    OutputInterface<double> angle_error_;

    double target_angle_ = 0.0;
    bool target_received_ = false;

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
