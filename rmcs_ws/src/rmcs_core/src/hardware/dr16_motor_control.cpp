#include <cstdint>
#include <memory>

// ROS2 Node。
// 后面会通过 YAML 参数读取 board_serial、PID 参数等。
#include <rclcpp/node.hpp>

// RMCS Component 基类。
// 继承以后，RMCS Executor 会周期调用 update()。
#include <rmcs_executor/component.hpp>

// DR16 遥控器解析。
#include "hardware/device/dr16.hpp"

// DJI 电机封装。
// 负责解析 M3508 的角度、转速、力矩、温度，
// 后面还会负责生成 C620 的控制指令。
#include "hardware/device/dji_motor.hpp"

// 我们实际使用的开发板。
// lsusb 已经确认：
// VID = a11c
// PID = d401
// 对应 librmcs::board::CBoard。
#include "librmcs/board/c_board.hpp"


namespace rmcs_core::hardware {

// ============================================================
// 第二周任务二：DR16 + M3508 电机控制 Hardware
// ============================================================
//
// 当前已经完成：
//
// DR16
//   ↓ DBUS
// CBoard
//   ↓
// uart_receive_callback()
//   ↓
// dr16_.store_status()
//   ↓
// dr16_.update_status()
//
//
// M3508 + C620(ID = 3)
//   ↓ CAN1 / 0x203
// CBoard
//   ↓
// can_receive_callback()
//   ↓
// motor_.match_then_store_status()
//   ↓
// motor_.update_status()
//   ↓
// motor_.velocity()
// motor_.angle()
// motor_.temperature()
//
//
// 当前还没有做：
//
// DR16 摇杆
//      ↓
// 目标速度
//      ↓
// LowPassFilter
//      ↓
// PID
//      ↓
// control_torque
//      ↓
// CAN 控制发送
//
// 所以当前版本依旧只是：
//
// “读取电机反馈”
//
// 不会主动让 M3508 转动。
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
        // 配置 M3508
        // ========================================================
        //
        // 我们已经实际测试得到：
        //
        // M3508 feedback connected:
        // CAN1, ID = 0x203
        //
        // 对于 M3508：
        //
        // 接收反馈 CAN ID
        // = 0x200 + 电调 ID
        //
        // 所以：
        //
        // 0x203
        //   ↓
        // C620 ID = 3
        //
        // 因此这里正式使用：
        //
        // M3508
        // ID 3
        motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kM3508,
                3
            }
        );


        // ========================================================
        // 创建 CBoard 通信对象
        // ========================================================
        board_ = std::make_unique<librmcs::board::CBoard>(
            *this,
            get_parameter("board_serial").as_string()
        );

        // 第一个参数 *this：
        //
        // 把当前 Dr16MotorControl 当成 CBoard Callback。
        //
        // CBoard 收到：
        //
        // DBUS
        // CAN
        //
        // 数据后，就会调用：
        //
        // uart_receive_callback()
        // can_receive_callback()
        //
        //
        // 第二个参数 board_serial：
        //
        // 当前 YAML 中：
        //
        // board_serial: ""
        //
        // 空字符串表示不按照序列号筛选 CBoard。
    }


    // ============================================================
    // RMCS 周期更新
    // ============================================================
    void update() override {

        // --------------------------------------------------------
        // 1. 更新 DR16 状态
        // --------------------------------------------------------
        dr16_.update_status();

        // uart_receive_callback()
        // 负责保存原始 DBUS 数据。
        //
        // update_status()
        // 再把它解析成：
        //
        // joystick_left()
        // joystick_right()
        // 拨杆
        // 键盘
        // 鼠标
        //
        // 后面“摇杆 → 目标速度”会使用这里的数据。


        // --------------------------------------------------------
        // 2. 更新 M3508 状态
        // --------------------------------------------------------
        motor_.update_status();

        // can_receive_callback()
        // 保存的是原始 8 Byte CAN 数据。
        //
        // motor_.update_status()
        // 会把这些数据解析成：
        //
        // angle()       rad
        // velocity()    rad/s
        // torque()      N·m
        // temperature() ℃


        // --------------------------------------------------------
        // 3. 还没有收到真实 M3508 反馈时，不打印
        // --------------------------------------------------------
        if (!motor_feedback_found_) {
            return;
        }


        // --------------------------------------------------------
        // 4. 控制日志打印频率
        // --------------------------------------------------------
        ++velocity_log_counter_;

        // 当前 YAML：
        //
        // update_rate: 1000.0
        //
        // 也就是：
        //
        // 每秒大约执行 1000 次 update()
        //
        // 如果我们每次都打印：
        //
        // 一秒钟会产生大约 1000 行日志，
        // 终端会完全被刷满。
        //
        // 所以累计 1000 次以后才打印一次。
        if (velocity_log_counter_ >= 1000) {

            velocity_log_counter_ = 0;

            RCLCPP_INFO(
                get_logger(),

                "M3508 velocity = %.3f rad/s, "
                "angle = %.3f rad, "
                "temperature = %.1f C",

                motor_.velocity(),
                motor_.angle(),
                motor_.temperature()
            );

            // ----------------------------------------------------
            // motor_.velocity()
            // ----------------------------------------------------
            //
            // 实际电机速度。
            //
            // 单位：
            //
            // rad/s
            //
            // 即“弧度每秒”。
            //
            // 1 圈 = 2π rad
            //
            // 所以：
            //
            // 60 RPM
            // = 每秒 1 圈
            // ≈ 6.283 rad/s
            //
            //
            // ----------------------------------------------------
            // motor_.angle()
            // ----------------------------------------------------
            //
            // 当前角度。
            //
            // 单位：
            //
            // rad
            //
            //
            // ----------------------------------------------------
            // motor_.temperature()
            // ----------------------------------------------------
            //
            // 电机反馈温度。
            //
            // 单位：
            //
            // 摄氏度 ℃
        }
    }


    // ============================================================
    // DR16 DBUS 接收回调
    // ============================================================
    void uart_receive_callback(
        const Spec::Uart& uart,
        const View::Uart& data
    ) override {

        // CBoard 当前具有：
        //
        // DBUS
        // UART1
        // UART2
        //
        // DR16 使用的是 DBUS，
        // 所以这里只处理 DBUS 数据。
        if (uart == Spec::kUarts.kDbus) {

            dr16_.store_status(
                data.uart_data.data(),
                data.uart_data.size()
            );

            // data.uart_data.data()
            //
            // UART 数据在内存中的起始地址。
            //
            //
            // data.uart_data.size()
            //
            // 当前 UART 数据长度。
            //
            //
            // store_status()
            //
            // 先保存原始 DR16 数据。
            //
            // 下一次 update()：
            //
            // dr16_.update_status()
            //
            // 再真正解析。
        }
    }


    // ============================================================
    // M3508 CAN 接收回调
    // ============================================================
    void can_receive_callback(
        const Spec::Can& can,
        const View::Can& data
    ) override {

        // --------------------------------------------------------
        // 1. 只监听物理 CAN1
        // --------------------------------------------------------
        //
        // 我们已经从 CBoard 源码确认：
        //
        // 物理 CAN1
        //     ↓
        // Spec::kCans.kCan1
        //
        // 同时实物 C620 也确实连接在 CAN1。
        if (can != Spec::kCans.kCan1) {
            return;
        }


        // --------------------------------------------------------
        // 2. 检查 CAN 帧格式
        // --------------------------------------------------------
        if (
            data.is_extended_can_id ||
            data.is_remote_transmission ||
            data.can_data.size() != 8
        ) {

            // M3508 + C620 正常反馈使用：
            //
            // 标准 CAN ID
            // 非 Remote Frame
            // 8 Byte 数据
            //
            // 不符合就不处理。
            return;
        }


        // --------------------------------------------------------
        // 3. 交给 DjiMotor 判断 CAN ID
        // --------------------------------------------------------
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
        // DjiMotor 内部计算：
        //
        // recv_id()
        // = 0x200 + 3
        // = 0x203
        //
        //
        // 如果收到：
        //
        // CAN ID = 0x203
        //
        // matched = true
        //
        // 数据会被保存。
        //
        //
        // 如果收到：
        //
        // 0x201
        // 0x202
        // 或者其他设备
        //
        // matched = false
        //
        // 不会污染当前 motor_ 的状态。


        // --------------------------------------------------------
        // 4. 第一次收到正确反馈时打印一次
        // --------------------------------------------------------
        if (matched && !motor_feedback_found_) {

            RCLCPP_INFO(
                get_logger(),
                "M3508 feedback connected: CAN1, ID = 0x%03X",
                static_cast<unsigned int>(data.can_id)
            );

            motor_feedback_found_ = true;
        }

        // C620 会持续发送大量反馈。
        //
        // 因此这个“连接成功”日志只打印一次。
    }


private:

    // ============================================================
    // M3508 是否已经连接成功
    // ============================================================
    bool motor_feedback_found_ = false;

    // false：
    //
    // 还没有收到 M3508 ID 3 的 0x203。
    //
    // true：
    //
    // 已经至少收到过一次正确反馈。


    // ============================================================
    // 转速日志计数器
    // ============================================================
    std::uint32_t velocity_log_counter_ = 0;

    // 当前 RMCS：
    //
    // update_rate = 1000 Hz
    //
    // 因此：
    //
    // velocity_log_counter_ += 1
    //
    // 累计到 1000：
    //
    // 大约经过 1 秒。
    //
    // 这样我们只会每秒打印一次电机反馈。


    // ============================================================
    // CBoard
    // ============================================================
    std::unique_ptr<librmcs::board::CBoard> board_;

    // 数据链：
    //
    // 电脑
    //   ↓ USB
    // CBoard
    //   ↓
    // DBUS / CAN
    //   ↓
    // 当前 Dr16MotorControl
    //
    //
    // 为什么用 unique_ptr？
    //
    // 因为 CBoard 构造以后可能立刻开始接收硬件数据。
    //
    // 我们希望：
    //
    // 其他成员先初始化
    //       ↓
    // 再创建 CBoard
    //
    // 这样更加安全。


    // ============================================================
    // DR16
    // ============================================================
    device::Dr16 dr16_;

    // 数据链：
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
    // update()
    //      ↓
    // dr16_.update_status()
    //
    // 后面会读取：
    //
    // dr16_.joystick_left()
    //
    // 来产生 M3508 目标速度。


    // ============================================================
    // M3508
    // ============================================================
    device::DjiMotor motor_{
        *this,
        *this,
        "/motor"
    };

    // DjiMotor 第一个 Component：
    //
    // *this
    //
    // 用来注册电机状态输出：
    //
    // /motor/angle
    // /motor/velocity
    // /motor/torque
    // /motor/max_torque
    //
    //
    // 第二个 Component：
    //
    // *this
    //
    // 用来注册：
    //
    // /motor/control_torque
    //
    //
    // 但是当前版本没有调用：
    //
    // board_->start_transmit()
    // can_transmit()
    // motor_.generate_command()
    //
    // 因此现在仍然只接收反馈，
    // 不会主动控制 M3508。
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

// 使 RMCS Executor 可以通过 pluginlib
// 动态创建 Dr16MotorControl。
