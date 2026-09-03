#include <cstdint>
#include <memory>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "filter/low_pass_filter.hpp"

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"

#include "librmcs/board/c_board.hpp"


namespace rmcs_core::hardware {

// ============================================================
// 第二周任务二：DR16 控制 M3508 速度闭环
// ============================================================
//
// 完整控制链：
//
// DR16 左摇杆 Y
//      ↓
// target_velocity_
//      ↓
// target - filtered
//      ↓
// velocity_error_
//      ↓
// PidCalculator
//      ↓
// control_torque_
//      ↓
// DjiMotor::generate_command()
//      ↓
// CAN1 / 0x200
//      ↓
// C620 ID = 3
//      ↓
// M3508
//
// 同时反馈链：
//
// M3508
//      ↓
// C620 0x203
//      ↓
// motor_.velocity()
//      ↓
// LowPassFilter
//      ↓
// filtered_velocity_
//      └──────────────→ 回到 PID
//
class Dr16MotorControl
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {

public:
    Dr16MotorControl()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}
                  .automatically_declare_parameters_from_overrides(true)
          } {

        // --------------------------------------------------------
        // 1. 读取 DR16 最大目标速度
        // --------------------------------------------------------
        max_target_velocity_ =
            get_parameter("max_target_velocity").as_double();


        // --------------------------------------------------------
        // 2. 读取速度反馈低通滤波截止频率
        // --------------------------------------------------------
        velocity_filter_cutoff_hz_ =
            get_parameter("velocity_filter_cutoff_hz").as_double();


        // --------------------------------------------------------
        // 3. 配置速度反馈低通滤波器
        // --------------------------------------------------------
        velocity_filter_.set_cutoff(
            velocity_filter_cutoff_hz_,
            kUpdateFrequencyHz
        );

        // 参数顺序：
        //
        // set_cutoff(
        //     截止频率,
        //     采样频率
        // )
        //
        // 当前：
        //
        // cutoff   = 10 Hz
        // sampling = 1000 Hz


        // --------------------------------------------------------
        // 4. 创建速度 PID
        // --------------------------------------------------------
        velocity_pid_ =
            controller::pid::make_pid_calculator(
                *this,
                "velocity_"
            );

        // 会从 YAML 读取：
        //
        // velocity_kp
        // velocity_ki
        // velocity_kd
        //
        // velocity_integral_min
        // velocity_integral_max
        //
        // velocity_integral_split_min
        // velocity_integral_split_max
        //
        // velocity_output_min
        // velocity_output_max


        // --------------------------------------------------------
        // 5. 配置 M3508
        // --------------------------------------------------------
        motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kM3508,
                3
            }
        );

        // 实机已经确认：
        //
        // C620 ID = 3
        //
        // 反馈：
        //
        // CAN1
        // CAN ID = 0x203


        // --------------------------------------------------------
        // 6. 创建 CBoard
        // --------------------------------------------------------
        board_ = std::make_unique<librmcs::board::CBoard>(
            *this,
            get_parameter("board_serial").as_string()
        );
    }


    // ============================================================
    // RMCS 主更新函数
    // ============================================================
    void update() override {

        // --------------------------------------------------------
        // 1. 更新 DR16
        // --------------------------------------------------------
        dr16_.update_status();


        // --------------------------------------------------------
        // 2. DR16 → 目标速度
        // --------------------------------------------------------
        if (dr16_.valid()) {

            target_velocity_ =
                dr16_.joystick_left().y()
                * max_target_velocity_;

        } else {

            // 遥控器掉线：
            //
            // 目标速度立即清零。
            target_velocity_ = 0.0;
        }


        // --------------------------------------------------------
        // 3. 更新 M3508 反馈状态
        // --------------------------------------------------------
        motor_.update_status();


        // --------------------------------------------------------
        // 4. M3508 原始速度 → 低通滤波
        // --------------------------------------------------------
        if (motor_feedback_found_) {

            raw_velocity_ =
                motor_.velocity();

            filtered_velocity_ =
                velocity_filter_.update(
                    raw_velocity_
                );

        } else {

            // 没有真实反馈时，
            // 不允许使用旧速度。
            raw_velocity_ = 0.0;
            filtered_velocity_ = 0.0;

            velocity_filter_.reset();
        }


        // --------------------------------------------------------
        // 5. PID 闭环计算
        // --------------------------------------------------------
        //
        // 只有：
        //
        // DR16 在线
        // AND
        // M3508 反馈在线
        //
        // 才允许产生非零控制量。
        if (
            dr16_.valid()
            && motor_feedback_found_
        ) {

            // error = target - actual
            velocity_error_ =
                target_velocity_
                - filtered_velocity_;


            // PID 根据误差计算目标控制力矩。
            control_torque_ =
                velocity_pid_.update(
                    velocity_error_
                );

        } else {

            // ----------------------------------------------------
            // 安全状态
            // ----------------------------------------------------
            //
            // DR16 掉线或者电机反馈丢失：
            //
            // 立即清零。
            velocity_error_ = 0.0;
            control_torque_ = 0.0;

            // 同时清除 PID 历史。
            velocity_pid_.reset();
        }


        // --------------------------------------------------------
        // 6. 发送 M3508 控制 CAN 帧
        // --------------------------------------------------------
        command_update();

        // 从这一版开始：
        //
        // control_torque_
        //
        // 不再只是一个打印出来的变量，
        // 而是真正会通过 CAN 发给 C620。


        // --------------------------------------------------------
        // 7. 每秒打印一次完整状态
        // --------------------------------------------------------
        ++status_log_counter_;

        if (status_log_counter_ >= 1000) {

            status_log_counter_ = 0;

            RCLCPP_INFO(
                get_logger(),

                "DR16=%s, "
                "joy=%.3f, "
                "target=%.3f, "
                "raw=%.3f, "
                "filtered=%.3f, "
                "error=%.3f, "
                "torque_cmd=%.3f",

                dr16_.valid()
                    ? "valid"
                    : "invalid",

                dr16_.joystick_left().y(),

                target_velocity_,

                raw_velocity_,

                filtered_velocity_,

                velocity_error_,

                control_torque_
            );
        }
    }


    // ============================================================
    // 发送 M3508 控制命令
    // ============================================================
    void command_update() {

        // --------------------------------------------------------
        // 新名词：start_transmit()
        // --------------------------------------------------------
        //
        // 它会创建一次发送事务。
        //
        // 我们已经从 flight.cpp 当前主线确认：
        //
        // auto builder = board_->start_transmit();
        // builder.can_transmit(...);
        //
        // 所以这里完全仿照主线。
        auto builder =
            board_->start_transmit();


        // --------------------------------------------------------
        // CAN1 发送 ID = 0x200
        // --------------------------------------------------------
        builder.can_transmit(
            Spec::kCans.kCan1,
            {
                .can_id = 0x200,

                .can_data =
                    device::CanPacket8{

                        // ----------------------------------------
                        // Quarter 1
                        // 对应 C620 ID = 1
                        // ----------------------------------------
                        device::CanPacket8::PaddingQuarter{},

                        // ----------------------------------------
                        // Quarter 2
                        // 对应 C620 ID = 2
                        // ----------------------------------------
                        device::CanPacket8::PaddingQuarter{},

                        // ----------------------------------------
                        // Quarter 3
                        // 对应 C620 ID = 3
                        // ----------------------------------------
                        //
                        // 我们的 M3508 就是 ID 3。
                        //
                        // generate_command(control_torque_)
                        //
                        // 会把控制力矩转换成
                        // C620 所需要的原始控制值。
                        motor_.generate_command(
                            control_torque_
                        ),

                        // ----------------------------------------
                        // Quarter 4
                        // 对应 C620 ID = 4
                        // ----------------------------------------
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            }
        );
    }


    // ============================================================
    // DR16 / DBUS 接收
    // ============================================================
    void uart_receive_callback(
        const Spec::Uart& uart,
        const View::Uart& data
    ) override {

        if (uart != Spec::kUarts.kDbus) {
            return;
        }


        // 第一次收到 DBUS 时提示。
        if (!dbus_packet_found_) {

            RCLCPP_INFO(
                get_logger(),
                "DBUS packet received, size = %zu bytes",
                data.uart_data.size()
            );

            dbus_packet_found_ = true;
        }


        // 保存 DR16 原始数据。
        dr16_.store_status(
            data.uart_data.data(),
            data.uart_data.size()
        );
    }


    // ============================================================
    // M3508 CAN 反馈接收
    // ============================================================
    void can_receive_callback(
        const Spec::Can& can,
        const View::Can& data
    ) override {

        // 当前 M3508 接在 CAN1。
        if (can != Spec::kCans.kCan1) {
            return;
        }


        // DJI 正常反馈应当是：
        //
        // 标准 CAN
        // 非 Remote Frame
        // 8 Byte
        if (
            data.is_extended_can_id
            || data.is_remote_transmission
            || data.can_data.size() != 8
        ) {
            return;
        }


        // --------------------------------------------------------
        // 判断是不是当前 M3508
        // --------------------------------------------------------
        const bool matched =
            motor_.match_then_store_status(
                data.can_id,
                data.can_data
            );

        // M3508 ID = 3：
        //
        // recv ID
        // =
        // 0x200 + 3
        // =
        // 0x203


        // --------------------------------------------------------
        // 第一次收到真实反馈
        // --------------------------------------------------------
        if (
            matched
            && !motor_feedback_found_
        ) {

            RCLCPP_INFO(
                get_logger(),
                "M3508 feedback connected: CAN1, ID = 0x%03X",
                static_cast<unsigned int>(
                    data.can_id
                )
            );

            motor_feedback_found_ = true;

            // 第一次建立反馈时，
            // 清空滤波和 PID 历史。
            velocity_filter_.reset();
            velocity_pid_.reset();
        }
    }


private:

    // ============================================================
    // RMCS 更新频率
    // ============================================================
    static constexpr double
        kUpdateFrequencyHz = 1000.0;


    // ============================================================
    // 最大目标速度
    // ============================================================
    double max_target_velocity_ = 0.0;


    // ============================================================
    // 当前目标速度
    // ============================================================
    double target_velocity_ = 0.0;


    // ============================================================
    // 速度低通滤波截止频率
    // ============================================================
    double velocity_filter_cutoff_hz_ = 0.0;


    // ============================================================
    // 原始速度
    // ============================================================
    double raw_velocity_ = 0.0;


    // ============================================================
    // 滤波后速度
    // ============================================================
    double filtered_velocity_ = 0.0;


    // ============================================================
    // 速度误差
    // ============================================================
    double velocity_error_ = 0.0;

    // velocity_error_
    // =
    // target_velocity_
    // -
    // filtered_velocity_


    // ============================================================
    // PID 输出 / 电机控制力矩
    // ============================================================
    double control_torque_ = 0.0;

    // 从这一版开始：
    //
    // 这个变量真正送进：
    //
    // motor_.generate_command(...)
    //
    // 然后通过 CAN 发给 C620。


    // ============================================================
    // LowPassFilter
    // ============================================================
    filter::LowPassFilter<1>
        velocity_filter_{1.0};


    // ============================================================
    // 速度 PID
    // ============================================================
    controller::pid::PidCalculator
        velocity_pid_{};


    // ============================================================
    // DBUS 是否已经收到
    // ============================================================
    bool dbus_packet_found_ = false;


    // ============================================================
    // M3508 是否已经收到反馈
    // ============================================================
    bool motor_feedback_found_ = false;


    // ============================================================
    // 日志计数器
    // ============================================================
    std::uint32_t status_log_counter_ = 0;


    // ============================================================
    // CBoard
    // ============================================================
    std::unique_ptr<librmcs::board::CBoard>
        board_;


    // ============================================================
    // DR16
    // ============================================================
    device::Dr16 dr16_;


    // ============================================================
    // M3508
    // ============================================================
    device::DjiMotor motor_{
        *this,
        *this,
        "/motor"
    };
};

} // namespace rmcs_core::hardware


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::Dr16MotorControl,
    rmcs_executor::Component
)
