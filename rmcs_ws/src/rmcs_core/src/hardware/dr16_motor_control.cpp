#include <cstdint>
#include <memory>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"

#include "librmcs/board/c_board.hpp"


namespace rmcs_core::hardware {

// ============================================================
// 第二周任务二：DR16 + M3508 电机控制 Hardware
// ============================================================
//
// 当前已经完成：
//
// ① DR16 数据链
//
// DR16 遥控器
//      ↓ 无线
// DR16 接收机
//      ↓ DBUS
// CBoard
//      ↓
// uart_receive_callback()
//      ↓
// dr16_.store_status()
//      ↓
// dr16_.update_status()
//
//
// ② M3508 反馈链
//
// M3508
//      ↓
// C620，ID = 3
//      ↓ CAN1 / 0x203
// CBoard
//      ↓
// can_receive_callback()
//      ↓
// DjiMotor
//      ↓
// angle / velocity / torque / temperature
//
//
// ③ 当前新增
//
// DR16 左摇杆 Y
//      ↓
// [-1, +1]
//      ↓
// × max_target_velocity
//      ↓
// target_velocity
//
//
// 目前仍然没有：
//
// LowPassFilter
// PID
// CAN 控制发送
//
// 所以：
// 当前程序依旧不会主动控制 M3508 转动。
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

        // ========================================================
        // 读取最大目标速度
        // ========================================================
        //
        // YAML 中我们会写：
        //
        // max_target_velocity: 10.0
        //
        // 单位：
        //
        // rad/s
        //
        // 以后摇杆推满时：
        //
        // +1 × 10
        // = +10 rad/s
        //
        // 摇杆拉到底：
        //
        // -1 × 10
        // = -10 rad/s
        max_target_velocity_ =
            get_parameter("max_target_velocity").as_double();


        // ========================================================
        // 配置 M3508
        // ========================================================
        //
        // 我们已经通过真实硬件测试确认：
        //
        // M3508 feedback connected:
        // CAN1, ID = 0x203
        //
        // 对 M3508：
        //
        // recv_id = 0x200 + 电调 ID
        //
        // 所以：
        //
        // 0x203
        //   ↓
        // ID = 3
        motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kM3508,
                3
            }
        );


        // ========================================================
        // 创建 CBoard
        // ========================================================
        board_ = std::make_unique<librmcs::board::CBoard>(
            *this,
            get_parameter("board_serial").as_string()
        );

        // *this：
        //
        // 当前 Dr16MotorControl 本身就是
        // CBoard::Callback。
        //
        // 因此 CBoard 收到：
        //
        // DBUS
        // CAN
        //
        // 数据以后，会调用当前类的：
        //
        // uart_receive_callback()
        // can_receive_callback()
        //
        //
        // board_serial：
        //
        // 当前我们设置为空字符串：
        //
        // ""
        //
        // 意味着不按序列号筛选 CBoard。
    }


    // ============================================================
    // RMCS 周期更新函数
    // ============================================================
    void update() override {

        // ========================================================
        // 1. 更新 DR16
        // ========================================================
        dr16_.update_status();

        // uart_receive_callback()
        // 只是负责保存原始 DBUS 数据。
        //
        // update_status()
        // 才真正解析：
        //
        // 左摇杆
        // 右摇杆
        // 拨杆
        // 鼠标
        // 键盘
        // 等状态。


        // ========================================================
        // 2. DR16 左摇杆 Y → 目标速度
        // ========================================================
        if (dr16_.valid()) {

            // dr16_.valid() == true
            //
            // 表示：
            //
            // 当前遥控器数据有效。
            //
            //
            // joystick_left().y()
            //
            // 大致范围：
            //
            // -1.0 ～ +1.0
            //
            //
            // 所以：
            //
            // target_velocity
            // =
            // joystick_y × max_target_velocity

            target_velocity_ =
                dr16_.joystick_left().y()
                * max_target_velocity_;

        } else {

            // ====================================================
            // 遥控器掉线保护
            // ====================================================
            //
            // 如果 DR16 掉线，
            // 目标速度必须立即清零。
            //
            // 这是非常重要的安全逻辑。
            //
            // 假设之前：
            //
            // target_velocity = 10 rad/s
            //
            // 然后遥控器突然断开。
            //
            // 如果我们不清零：
            //
            // PID 以后仍然可能继续认为目标速度是 10，
            // 电机就可能继续转。
            //
            // 所以掉线以后：
            //
            // target_velocity = 0
            target_velocity_ = 0.0;
        }


        // ========================================================
        // 3. 更新 M3508
        // ========================================================
        motor_.update_status();

        // can_receive_callback()
        // 保存的是原始 CAN 数据。
        //
        // update_status()
        // 负责解析：
        //
        // angle()       rad
        // velocity()    rad/s
        // torque()      N·m
        // temperature() °C


        // ========================================================
        // 4. 每秒打印一次状态
        // ========================================================
        ++status_log_counter_;

        // 当前：
        //
        // update_rate = 1000 Hz
        //
        // 所以大约：
        //
        // 1000 次 update()
        // ≈ 1 秒
        if (status_log_counter_ >= 1000) {

            status_log_counter_ = 0;

            RCLCPP_INFO(
                get_logger(),

                "DR16 valid = %s, "
                "joystick Y = %.3f, "
                "target = %.3f rad/s, "
                "motor = %s, "
                "actual = %.3f rad/s",

                dr16_.valid() ? "true" : "false",

                dr16_.joystick_left().y(),

                target_velocity_,

                motor_feedback_found_
                    ? "connected"
                    : "disconnected",

                motor_.velocity()
            );

            // 比如以后可能看到：
            //
            // DR16 valid = true,
            // joystick Y = 0.500,
            // target = 5.000 rad/s,
            // motor = disconnected,
            // actual = 0.000 rad/s
            //
            //
            // 这意味着：
            //
            // 遥控器正常
            // 左摇杆推到一半
            // 目标速度 5 rad/s
            //
            // 但因为现在还没写 PID + CAN 控制，
            // 所以电机实际速度仍然是 0。
        }
    }


    // ============================================================
    // DR16 DBUS 接收
    // ============================================================
    void uart_receive_callback(
        const Spec::Uart& uart,
        const View::Uart& data
    ) override {

        // CBoard 具有：
        //
        // DBUS
        // UART1
        // UART2
        //
        // DR16 使用的是 DBUS。
        if (uart == Spec::kUarts.kDbus) {

            dr16_.store_status(
                data.uart_data.data(),
                data.uart_data.size()
            );

            // data()
            //
            // 当前 DBUS 数据的内存起始地址。
            //
            // size()
            //
            // 当前 DBUS 数据长度。
            //
            //
            // store_status()
            //
            // 把原始数据保存起来。
            //
            // 下一次 update()：
            //
            // dr16_.update_status()
            //
            // 再真正解析。
        }
    }


    // ============================================================
    // M3508 CAN 接收
    // ============================================================
    void can_receive_callback(
        const Spec::Can& can,
        const View::Can& data
    ) override {

        // ========================================================
        // 1. 只处理 CAN1
        // ========================================================
        //
        // 实物接线已经确认：
        //
        // C620
        //   ↓
        // CBoard CAN1
        //
        // CBoard 源码也已经确认：
        //
        // 物理 CAN1
        // =
        // Spec::kCans.kCan1
        if (can != Spec::kCans.kCan1) {
            return;
        }


        // ========================================================
        // 2. 检查 CAN 数据格式
        // ========================================================
        if (
            data.is_extended_can_id ||
            data.is_remote_transmission ||
            data.can_data.size() != 8
        ) {

            // DJI M3508 正常反馈是：
            //
            // 标准 CAN 帧
            // 非 Remote Frame
            // 8 Byte
            //
            // 不符合则直接忽略。
            return;
        }


        // ========================================================
        // 3. 交给 DjiMotor 判断是不是自己的反馈
        // ========================================================
        const bool matched =
            motor_.match_then_store_status(
                data.can_id,
                data.can_data
            );

        // 当前 motor_：
        //
        // Type = M3508
        // ID   = 3
        //
        // 所以：
        //
        // recv_id()
        // =
        // 0x200 + 3
        // =
        // 0x203
        //
        //
        // 收到 0x203：
        //
        // matched = true
        //
        // 数据被保存。
        //
        //
        // 收到其他 CAN ID：
        //
        // matched = false
        //
        // 不处理。


        // ========================================================
        // 4. 第一次连接成功时打印
        // ========================================================
        if (matched && !motor_feedback_found_) {

            RCLCPP_INFO(
                get_logger(),
                "M3508 feedback connected: CAN1, ID = 0x%03X",
                static_cast<unsigned int>(data.can_id)
            );

            motor_feedback_found_ = true;
        }
    }


private:

    // ============================================================
    // 最大目标速度
    // ============================================================
    double max_target_velocity_ = 0.0;

    // 单位：
    //
    // rad/s
    //
    // 来自 YAML：
    //
    // max_target_velocity: 10.0


    // ============================================================
    // 当前目标速度
    // ============================================================
    double target_velocity_ = 0.0;

    // 数据链：
    //
    // 左摇杆 Y
    //      ↓
    // [-1, +1]
    //      ↓
    // × max_target_velocity
    //      ↓
    // target_velocity_
    //
    // 单位：
    //
    // rad/s


    // ============================================================
    // 是否收到过 M3508 反馈
    // ============================================================
    bool motor_feedback_found_ = false;

    // false：
    //
    // 还没有收到 0x203。
    //
    // true：
    //
    // 已经收到过真实 M3508 反馈。


    // ============================================================
    // 状态日志计数器
    // ============================================================
    std::uint32_t status_log_counter_ = 0;

    // RMCS 1000 Hz：
    //
    // 每执行一次 update()
    // 就 +1。
    //
    // 到 1000 后打印一次，
    // 相当于大约每秒打印一次。


    // ============================================================
    // CBoard
    // ============================================================
    std::unique_ptr<librmcs::board::CBoard> board_;

    // 电脑
    //   ↓ USB
    // CBoard
    //   ↓
    // DBUS / CAN
    //   ↓
    // Dr16MotorControl


    // ============================================================
    // DR16
    // ============================================================
    device::Dr16 dr16_;

    // 遥控器
    //   ↓
    // DR16 接收机
    //   ↓ DBUS
    // CBoard
    //   ↓
    // dr16_


    // ============================================================
    // M3508
    // ============================================================
    device::DjiMotor motor_{
        *this,
        *this,
        "/motor"
    };

    // 第一个 *this：
    //
    // 注册电机状态输出：
    //
    // /motor/angle
    // /motor/velocity
    // /motor/torque
    // /motor/max_torque
    //
    //
    // 第二个 *this：
    //
    // 注册：
    //
    // /motor/control_torque
    //
    //
    // 但当前版本仍然没有：
    //
    // board_->start_transmit()
    // can_transmit()
    // motor_.generate_command()
    //
    // 所以现在不会主动控制电机。
};

} // namespace rmcs_core::hardware


// ============================================================
// pluginlib 导出
// ============================================================

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::Dr16MotorControl,
    rmcs_executor::Component
)