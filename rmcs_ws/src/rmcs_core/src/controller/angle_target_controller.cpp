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

        // 参考 simple_gimbal_controller.cpp：读取角度反馈、输出角度误差
        register_input("/motor/angle", motor_angle_);

        register_output(
            "/motor/control_angle_error",
            angle_error_,
            nan_
        );


        // 参考 omni_infantry.cpp 中 create_subscription() 的写法
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

        // 没收到目标角度以前不启动位置控制
        if (!target_received_) {
            *angle_error_ = nan_;
            return;
        }


        // 参考 RMCS 中 std::remainder() 的角度环绕处理
        // 将误差限制在 [-pi, pi]，让电机尽量走较短的方向
        *angle_error_ =
            std::remainder(
                target_angle_ - *motor_angle_,
                2.0 * std::numbers::pi
            );
    }


private:
    static constexpr double nan_ =
        std::numeric_limits<double>::quiet_NaN();


    // 来自 AngleMotorHardware
    InputInterface<double> motor_angle_;


    // 送给外环 ErrorPidController
    OutputInterface<double> angle_error_;


    double target_angle_ = 0.0;
    bool target_received_ = false;


    rclcpp::Subscription<
        std_msgs::msg::Float64
    >::SharedPtr target_angle_subscription_;
};

} // namespace rmcs_core::controller


#include <pluginlib/class_list_macros.hpp>

// 参考现有 controller Component 的 pluginlib 导出方式
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::AngleTargetController,
    rmcs_executor::Component
)