#include <cstdint>
#include <memory>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"

#include "librmcs/board/c_board.hpp"

namespace rmcs_core::hardware {

class AngleMotorHardware
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {

private:
    // 参考 omni_infantry.cpp，Partner Component 单独发送电机命令。
    class AngleMotorCommand : public rmcs_executor::Component {
    public:
        explicit AngleMotorCommand(AngleMotorHardware& hardware)
            : hardware_(hardware) {}

        void update() override {
            hardware_.command_update();
        }

    private:
        AngleMotorHardware& hardware_;
    };

public:
    AngleMotorHardware()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}
                  .automatically_declare_parameters_from_overrides(true)
          }

        , motor_command_(
              create_partner_component<AngleMotorCommand>(
                  get_component_name() + "_command",
                  *this
              )
          )
        , motor_{
              *this,
              *motor_command_,
              "/motor"
          } {

        // M3508（C620，ID=3），启用多圈角度。
        motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kM3508,
                3
            }
                .enable_multi_turn_angle()
        );

        // CBoard 连接方式沿用任务二。
        board_ =
            std::make_unique<librmcs::board::CBoard>(
                *this,
                get_parameter("board_serial").as_string()
            );
    }

    // 更新电机反馈状态。
    void update() override {
        // 统计反馈间隔，用于超时保护。
        if (
            motor_feedback_found_
            && motor_feedback_age_cycles_ <= kMotorFeedbackTimeoutCycles
        ) {
            ++motor_feedback_age_cycles_;
        }

        // 更新 angle、velocity 和 torque。
        motor_.update_status();

        const bool motor_feedback_alive =
            motor_feedback_found_
            && motor_feedback_age_cycles_ <= kMotorFeedbackTimeoutCycles;

        // 每约 1 秒打印一次状态。
        ++log_counter_;

        if (log_counter_ >= 1000) {
            log_counter_ = 0;

            RCLCPP_INFO(
                get_logger(),
                "feedback=%s, angle=%.3f rad, velocity=%.3f rad/s",
                motor_feedback_alive ? "alive" : "lost",
                motor_.angle(),
                motor_.velocity()
            );
        }
    }

    // 接收 M3508 CAN 反馈。
    void can_receive_callback(
        const Spec::Can& can,
        const View::Can& data
    ) override {

        if (can != Spec::kCans.kCan1)
            return;

        // 只处理 8 字节标准数据帧。
        if (
            data.is_extended_can_id
            || data.is_remote_transmission
            || data.can_data.size() != 8
        ) {
            return;
        }

        // 反馈匹配方式参考 dji_motor.hpp。
        const bool matched =
            motor_.match_then_store_status(
                data.can_id,
                data.can_data
            );

        if (!matched)
            return;

        // 收到反馈后清零超时计数。
        motor_feedback_age_cycles_ = 0;

        // 首帧反馈到来时，以启动位置校准零点。
        if (!motor_feedback_found_) {

            motor_feedback_found_ = true;

            // calibrate_zero_point() 使用刚解析的原始角度。
            motor_.update_status();

            motor_.calibrate_zero_point();

            RCLCPP_INFO(
                get_logger(),
                "M3508 feedback connected and zero calibrated, CAN ID = 0x%03X",
                static_cast<unsigned int>(data.can_id)
            );
        }
    }

private:
    // 向 C620 发送控制命令。
    void command_update() {
        auto builder =
            board_->start_transmit();

        const bool motor_feedback_alive =
            motor_feedback_found_
            && motor_feedback_age_cycles_ <= kMotorFeedbackTimeoutCycles;

        builder.can_transmit(
            Spec::kCans.kCan1,
            {
                .can_id = 0x200,

                .can_data =
                    device::CanPacket8{

                        // C620 ID 1
                        device::CanPacket8::PaddingQuarter{},

                        // C620 ID 2
                        device::CanPacket8::PaddingQuarter{},

                        // C620 ID 3：反馈超时时强制发送 0 力矩。
                        motor_feedback_alive
                            ? motor_.generate_command()
                            : motor_.generate_command(0.0),

                        // C620 ID 4
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            }
        );
    }

    std::unique_ptr<
        librmcs::board::CBoard
    > board_;

    // Partner Component 读取 control_torque 并发送 CAN。
    std::shared_ptr<
        AngleMotorCommand
    > motor_command_;

    // 当前 Component 输出状态，Partner Component 输入控制力矩。
    device::DjiMotor motor_;

    bool motor_feedback_found_ = false;

    std::uint32_t motor_feedback_age_cycles_ = 0;

    std::uint32_t log_counter_ = 0;

    // 1000 Hz 下 100 个周期约为 100 ms。
    static constexpr std::uint32_t kMotorFeedbackTimeoutCycles = 100;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::AngleMotorHardware,
    rmcs_executor::Component
)
