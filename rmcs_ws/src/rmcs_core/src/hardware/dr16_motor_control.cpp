#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/float64.hpp>

#include "filter/low_pass_filter.hpp"

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"

#include "librmcs/board/c_board.hpp"

namespace rmcs_core::hardware {

class Dr16MotorControl
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
private:
    // 参考 omni_infantry.cpp 的 Partner Component 写法
    class Dr16MotorCommand : public rmcs_executor::Component {
    public:
        explicit Dr16MotorCommand(Dr16MotorControl& hardware)
            : hardware_(hardware) {}

        void update() override { hardware_.command_update(); }

    private:
        Dr16MotorControl& hardware_;
    };

public:
    Dr16MotorControl()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , motor_command_(create_partner_component<Dr16MotorCommand>(
              get_component_name() + "_command", *this))
        , motor_{*this, *motor_command_, "/motor"} {

        // 参考 pid_controller.cpp 的 setpoint / measurement / control 连接方式
        register_output("/motor/target_velocity", target_velocity_output_, kNan);
        register_output("/motor/velocity_filtered", filtered_velocity_output_, kNan);

        max_target_velocity_ = get_parameter("max_target_velocity").as_double();
        velocity_filter_cutoff_hz_ =
            get_parameter("velocity_filter_cutoff_hz").as_double();
        get_parameter_or("autotune_mode", autotune_mode_, false);

        if (autotune_mode_) {
            autotune_target_subscription_ =
                create_subscription<std_msgs::msg::Float64>(
                    "/motor/autotune_target_velocity",
                    rclcpp::QoS{1}.reliable(),
                    [this](std_msgs::msg::Float64::UniquePtr&& msg) {
                        if (!std::isfinite(msg->data)) {
                            autotune_target_received_ = false;
                            return;
                        }
                        autotune_target_velocity_ = std::clamp(
                            msg->data, -max_target_velocity_, max_target_velocity_);
                        autotune_target_age_cycles_ = 0;
                        autotune_target_received_ = true;
                    });
        }

        // 参考 deformable_suspension.cpp 的 LowPassFilter 用法
        velocity_filter_.set_cutoff(velocity_filter_cutoff_hz_, kUpdateFrequencyHz);

        // 当前实机使用 M3508 + C620，C620 ID 为 3。
        motor_.configure(device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3});

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_parameter("board_serial").as_string());
    }

    void update() override {
        // 参考 flight.cpp 的 DR16 数据处理方式
        dr16_.update_status();

        if (autotune_mode_) {
            if (autotune_target_received_
                && autotune_target_age_cycles_ <= kAutotuneTargetTimeoutCycles) {
                ++autotune_target_age_cycles_;
            }
            target_velocity_ = autotune_target_alive() ? autotune_target_velocity_ : 0.0;
            *target_velocity_output_ = target_velocity_;
        } else if (dr16_.valid()) {
            const double joystick_y = dr16_.joystick_left().y();

            // 左摇杆 Y 映射目标速度，5% 死区内视为回中。
            target_velocity_ = std::abs(joystick_y) < kJoystickDeadzone
                ? 0.0
                : joystick_y * max_target_velocity_;
            *target_velocity_output_ = target_velocity_;
        } else {
            target_velocity_ = 0.0;
            *target_velocity_output_ = kNan;
        }

        if (motor_feedback_found_
            && motor_feedback_age_cycles_ <= kMotorFeedbackTimeoutCycles) {
            ++motor_feedback_age_cycles_;
        }

        // 连续约 100 ms 没有新反馈时，反馈视为丢失。
        const bool motor_feedback_alive =
            motor_feedback_found_
            && motor_feedback_age_cycles_ <= kMotorFeedbackTimeoutCycles;

        motor_.update_status();

        if (motor_feedback_alive) {
            raw_velocity_ = motor_.velocity();
            filtered_velocity_ = velocity_filter_.update(raw_velocity_);
            *filtered_velocity_output_ = filtered_velocity_;
        } else {
            // 不把丢失前的最后一次速度继续交给 PID。
            raw_velocity_ = 0.0;
            filtered_velocity_ = 0.0;
            *filtered_velocity_output_ = kNan;
            velocity_filter_.reset();
        }

        if (++status_log_counter_ >= 1000) {
            status_log_counter_ = 0;
            RCLCPP_INFO(
                get_logger(),
                "DR16=%s, feedback=%s, joy=%.3f, target=%.3f, raw=%.3f, filtered=%.3f",
                dr16_.valid() ? "valid" : "invalid",
                motor_feedback_alive ? "alive" : "lost",
                dr16_.joystick_left().y(), target_velocity_, raw_velocity_, filtered_velocity_);
        }
    }

    void command_update() {
        // 参考 flight.cpp 的 CAN 命令发送方式
        auto builder = board_->start_transmit();

        const bool motor_feedback_alive =
            motor_feedback_found_
            && motor_feedback_age_cycles_ <= kMotorFeedbackTimeoutCycles;
        const bool target_source_alive =
            autotune_mode_ ? autotune_target_alive() : dr16_.valid();
        const bool control_allowed = target_source_alive && motor_feedback_alive;

        // 安全门控不通过时，显式向 C620 发送零控制量。
        builder.can_transmit(
            Spec::kCans.kCan1,
            {
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        // 参考 dji_motor.hpp 的 control_torque Interface
                        control_allowed ? motor_.generate_command()
                                        : motor_.generate_command(0.0),
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            });
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart != Spec::kUarts.kDbus)
            return;

        if (!dbus_packet_found_) {
            RCLCPP_INFO(
                get_logger(), "DBUS packet received, size = %zu bytes", data.uart_data.size());
            dbus_packet_found_ = true;
        }

        dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (can != Spec::kCans.kCan1)
            return;

        if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() != 8)
            return;

        if (!motor_.match_then_store_status(data.can_id, data.can_data))
            return;

        // 新反馈将超时计数清零。
        motor_feedback_age_cycles_ = 0;

        if (!motor_feedback_found_) {
            RCLCPP_INFO(
                get_logger(), "M3508 feedback connected: CAN1, ID = 0x%03X",
                static_cast<unsigned int>(data.can_id));
            motor_feedback_found_ = true;
            velocity_filter_.reset();
        }
    }

private:
    bool autotune_target_alive() const {
        return autotune_target_received_
            && autotune_target_age_cycles_ <= kAutotuneTargetTimeoutCycles;
    }

    static constexpr double kUpdateFrequencyHz = 1000.0;
    static constexpr double kJoystickDeadzone = 0.05;
    static constexpr std::uint32_t kMotorFeedbackTimeoutCycles = 100;
    static constexpr std::uint32_t kAutotuneTargetTimeoutCycles = 200;
    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    double max_target_velocity_ = 0.0;
    double target_velocity_ = 0.0;
    double velocity_filter_cutoff_hz_ = 0.0;
    double raw_velocity_ = 0.0;
    double filtered_velocity_ = 0.0;
    double autotune_target_velocity_ = 0.0;

    filter::LowPassFilter<1> velocity_filter_{1.0};

    OutputInterface<double> target_velocity_output_;
    OutputInterface<double> filtered_velocity_output_;

    bool dbus_packet_found_ = false;
    bool motor_feedback_found_ = false;
    bool autotune_mode_ = false;
    bool autotune_target_received_ = false;
    std::uint32_t autotune_target_age_cycles_ = kAutotuneTargetTimeoutCycles + 1;
    std::uint32_t motor_feedback_age_cycles_ = kMotorFeedbackTimeoutCycles + 1;
    std::uint32_t status_log_counter_ = 0;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr autotune_target_subscription_;
    std::unique_ptr<librmcs::board::CBoard> board_;
    device::Dr16 dr16_;
    std::shared_ptr<Dr16MotorCommand> motor_command_;
    device::DjiMotor motor_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Dr16MotorControl, rmcs_executor::Component)
