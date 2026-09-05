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
    // 参考 omni_infantry.cpp：用 Partner Component 单独负责发送电机命令
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

        // 参考 omni_infantry.cpp 的 Partner Component 创建方式
        , motor_command_(
              create_partner_component<AngleMotorCommand>(
                  get_component_name() + "_command",
                  *this
              )
          )

        // 当前 Component 输出电机状态，Partner Component 接收控制力矩
        , motor_{
              *this,
              *motor_command_,
              "/motor"
          } {

        // M3508 + C620，ID = 3
        // 开启多圈角度，后面用于位置闭环
        motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kM3508,
                3
            }
                .enable_multi_turn_angle()
        );


        // CBoard 连接方式沿用任务二
        board_ =
            std::make_unique<librmcs::board::CBoard>(
                *this,
                get_parameter("board_serial").as_string()
            );
    }


    // ============================================================
    // RMCS 主循环：更新电机反馈状态
    // ============================================================
    void update() override {

        // 参考任务二：统计距离最后一帧反馈过去了多少个控制周期
        if (
            motor_feedback_found_
            && motor_feedback_age_cycles_ <= kMotorFeedbackTimeoutCycles
        ) {
            ++motor_feedback_age_cycles_;
        }


        // DjiMotor 会自动更新：
        // /motor/angle
        // /motor/velocity
        // /motor/torque
        motor_.update_status();


        const bool motor_feedback_alive =
            motor_feedback_found_
            && motor_feedback_age_cycles_ <= kMotorFeedbackTimeoutCycles;


        // 每约 1 秒打印一次状态
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


    // ============================================================
    // M3508 CAN 反馈接收
    // ============================================================
    void can_receive_callback(
        const Spec::Can& can,
        const View::Can& data
    ) override {

        // 当前电机接在 CBoard CAN1
        if (can != Spec::kCans.kCan1)
            return;


        // 只接受正常的 8 字节标准 CAN 数据帧
        if (
            data.is_extended_can_id
            || data.is_remote_transmission
            || data.can_data.size() != 8
        ) {
            return;
        }


        // 参考 dji_motor.hpp：判断是不是当前 M3508 的反馈
        const bool matched =
            motor_.match_then_store_status(
                data.can_id,
                data.can_data
            );


        if (!matched)
            return;


        // 收到新反馈，超时计数重新清零
        motor_feedback_age_cycles_ = 0;


        // 第一帧反馈到来时进行一次启动零点校准
        if (!motor_feedback_found_) {

            motor_feedback_found_ = true;


            // calibrate_zero_point() 使用 last_raw_angle_
            // 所以必须先解析一次刚收到的 CAN 数据
            motor_.update_status();


            // 将“程序启动时的当前位置”定义为 0 rad
            motor_.calibrate_zero_point();


            RCLCPP_INFO(
                get_logger(),
                "M3508 feedback connected and zero calibrated, CAN ID = 0x%03X",
                static_cast<unsigned int>(data.can_id)
            );
        }
    }


private:
    // ============================================================
    // 向 C620 发送控制命令
    // ============================================================
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

                        // C620 ID 3：当前 M3508
                        //
                        // 反馈正常：
                        // 从 /motor/control_torque 读取 PID 输出
                        //
                        // 反馈超时：
                        // 强制发送 0 力矩
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


    // Partner Component：负责读取 /motor/control_torque 并发送 CAN
    std::shared_ptr<
        AngleMotorCommand
    > motor_command_;


    // DjiMotor：
    // 当前 Component 输出 angle / velocity
    // Partner Component 输入 control_torque
    device::DjiMotor motor_;


    // 是否至少收到过一帧电机反馈
    bool motor_feedback_found_ = false;


    // 距离最后一帧电机反馈过去的控制周期数
    std::uint32_t motor_feedback_age_cycles_ = 0;


    std::uint32_t log_counter_ = 0;


    // update_rate = 1000 Hz
    // 100 个周期约等于 100 ms
    static constexpr std::uint32_t kMotorFeedbackTimeoutCycles = 100;
};

} // namespace rmcs_core::hardware


#include <pluginlib/class_list_macros.hpp>

// 注册为 RMCS Component
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::AngleMotorHardware,
    rmcs_executor::Component
)