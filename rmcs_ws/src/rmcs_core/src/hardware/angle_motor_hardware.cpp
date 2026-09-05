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
    // 参考 omni_infantry.cpp 中 InfantryCommand 的写法
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
        // 参考 omni_infantry.cpp 中 hardware Component 的 Node 初始化方式
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}
                  .automatically_declare_parameters_from_overrides(true)
          }

        // 参考 omni_infantry.cpp 的 Partner Component 创建方式
        , motor_command_(
              create_partner_component<AngleMotorCommand>(
                  get_component_name() + "_command",
                  *this
              )
          )

        // 参考 omni_infantry.cpp 中 DjiMotor 的 status / command 分离写法
        , motor_{
              *this,
              *motor_command_,
              "/motor"
          } {

        // 参考 dji_motor.hpp 和 omni_infantry.cpp 的电机配置方式
        motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kM3508,
                3
            }
                .enable_multi_turn_angle()
        );


        // CBoard 的连接方式沿用任务二
        board_ =
            std::make_unique<librmcs::board::CBoard>(
                *this,
                get_parameter("board_serial").as_string()
            );
    }


    void update() override {

        // 参考 dji_motor.hpp：更新后会自动输出 /motor/angle 和 /motor/velocity
        motor_.update_status();


        // 当前阶段留一个简单日志，方便确认角度和速度反馈
        ++log_counter_;

        if (log_counter_ >= 1000) {
            log_counter_ = 0;

            RCLCPP_INFO(
                get_logger(),
                "feedback=%s, angle=%.3f rad, velocity=%.3f rad/s",
                motor_feedback_found_ ? "yes" : "no",
                motor_.angle(),
                motor_.velocity()
            );
        }
    }


    // 参考任务二 dr16_motor_control.cpp 中的 CAN 接收方式
    void can_receive_callback(
        const Spec::Can& can,
        const View::Can& data
    ) override {

        if (can != Spec::kCans.kCan1)
            return;


        if (
            data.is_extended_can_id
            || data.is_remote_transmission
            || data.can_data.size() != 8
        )
            return;


        // 参考 dji_motor.hpp 的 match_then_store_status()
        const bool matched =
            motor_.match_then_store_status(
                data.can_id,
                data.can_data
            );


        if (matched && !motor_feedback_found_) {
            motor_feedback_found_ = true;

            RCLCPP_INFO(
                get_logger(),
                "M3508 feedback connected, CAN ID = 0x%03X",
                static_cast<unsigned int>(data.can_id)
            );
        }
    }


private:
    // 参考任务二 command_update()，这里只负责把 control_torque 发给 C620
    void command_update() {

        auto builder =
            board_->start_transmit();


        builder.can_transmit(
            Spec::kCans.kCan1,
            {
                .can_id = 0x200,

                .can_data =
                    device::CanPacket8{

                        device::CanPacket8::PaddingQuarter{},

                        device::CanPacket8::PaddingQuarter{},

                        // ID = 3，所以控制量放在第三个 Quarter
                        motor_.generate_command(),

                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            }
        );
    }


    std::unique_ptr<
        librmcs::board::CBoard
    > board_;


    // Partner Component，负责执行 command_update()
    std::shared_ptr<
        AngleMotorCommand
    > motor_command_;


    // DjiMotor 会在当前 Component 输出状态，在 Partner 中读取 control_torque
    device::DjiMotor motor_;


    bool motor_feedback_found_ = false;

    std::uint32_t log_counter_ = 0;
};

} // namespace rmcs_core::hardware


#include <pluginlib/class_list_macros.hpp>

// 参考现有 RMCS hardware 的 pluginlib 导出方式
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::AngleMotorHardware,
    rmcs_executor::Component
)