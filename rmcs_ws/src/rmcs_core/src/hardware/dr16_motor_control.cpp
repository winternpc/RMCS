#include <memory>
// std::unique_ptr 和 std::make_unique 的定义。
// 后面用它保存 CBoard 对象。

#include <rclcpp/node.hpp>
// ROS2 的 Node 定义。
// 后面会从 YAML / ROS2 参数中读取开发板序列号、
// 电机参数、PID 参数等。

#include <rmcs_executor/component.hpp>
// RMCS 的 Component 基类。
// 继承它之后，Dr16MotorControl 才能被 RMCS Executor 周期调用。

#include "hardware/device/dr16.hpp"
// RMCS 已经提供好的 DR16 遥控器解析类。
// DBUS 原始数据会先交给它保存，再由 update_status() 解析。

#include "librmcs/board/c_board.hpp"
// CBoard 的主机端接口。
//
// 我们已经通过 lsusb 实际确认：
//     VID = a11c
//     PID = d401
//
// 而 CBoard 源码中使用的也是：
//     0xA11C : 0xD401
//
// 所以我们手上的实物应该使用 CBoard，
// 而不是之前误用的 RmcsBoardLite。

namespace rmcs_core::hardware {

// 这是我们为了第二周任务二编写的 hardware。
//
// 当前阶段已经完成：
// 1. RMCS Component
// 2. ROS2 Node
// 3. CBoard Callback
// 4. CBoard USB 通信对象
// 5. DR16 DBUS 数据接收
// 6. 临时 CAN 电机反馈诊断
//
// 暂时还没有正式加入：
// 1. DjiMotor
// 2. 摇杆 → 目标速度
// 3. LowPassFilter
// 4. PidCalculator
//
// 当前 CAN 回调中的日志只是临时诊断功能，
// 用来确认：
// 1. C620 当前 ID
// 2. CAN1 是否能够收到电机反馈。
class Dr16MotorControl
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {

public:
    Dr16MotorControl()
        : Node{
              get_component_name(),
              // 使用 RMCS Component 的名字作为 ROS2 Node 名字。

              rclcpp::NodeOptions{}
                  .automatically_declare_parameters_from_overrides(true)
              // 允许 YAML 中传入的参数自动声明。
              //
              // 例如：
              //
              // dr16_motor_control:
              //   ros__parameters:
              //     board_serial: ""
              //
              // 后面就可以直接：
              //
              // get_parameter("board_serial")
              //
              // 读取这个参数。
          } {

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this,
            get_parameter("board_serial").as_string()
        );

        // ↑ 创建真正与 CBoard 通信的对象。
        //
        // 第一个参数 *this：
        // --------------------------------
        // 把当前 Dr16MotorControl 自己作为 Callback
        // 交给 CBoard。
        //
        // 数据链可以理解成：
        //
        // CBoard 收到 CAN / UART 数据
        //             ↓
        // librmcs
        //             ↓
        // 找到 Callback
        //             ↓
        // Dr16MotorControl
        //             ↓
        // uart_receive_callback()
        // 或
        // can_receive_callback()
        //
        //
        // 第二个参数 board_serial：
        // --------------------------------
        // 从 ROS2 参数中读取开发板序列号筛选条件。
        //
        // 我们当前 YAML 中写的是：
        //
        // board_serial: ""
        //
        // librmcs 源码已经确认：
        //
        // 空字符串表示不按照序列号筛选，
        // 即接受任意匹配的 CBoard。
    }

    void update() override {
        dr16_.update_status();

        // uart_receive_callback() 负责：
        //
        // 收到原始 DBUS 数据
        //        ↓
        // dr16_.store_status()
        //
        // 而这里的 update_status() 负责真正把原始数据解析成：
        //
        // 左摇杆
        // 右摇杆
        // 拨杆
        // 鼠标
        // 键盘
        //
        // RMCS Executor 会周期调用本函数。
        //
        // 当前 YAML 中：
        //
        // update_rate: 1000.0
        //
        // 即目标更新频率为 1000 Hz。
    }

    void uart_receive_callback(
        const Spec::Uart& uart,
        const View::Uart& data
    ) override {

        // CBoard 一共有：
        //
        // DBUS
        // UART1
        // UART2
        //
        // DR16 使用的是 DBUS，
        // 因此首先判断当前数据是不是来自 DBUS。

        if (uart == Spec::kUarts.kDbus) {

            dr16_.store_status(
                data.uart_data.data(),
                data.uart_data.size()
            );

            // uart_data.data()
            // --------------------------------
            // UART 数据在内存中的起始地址。
            //
            // uart_data.size()
            // --------------------------------
            // 当前这一帧 UART 数据的字节数。
            //
            // dr16_.store_status()
            // --------------------------------
            // 先把 DR16 原始 DBUS 数据保存起来。
            //
            // 下一次 update()：
            //
            // dr16_.update_status()
            //
            // 才会进一步把这些数据解析成摇杆等状态。
        }
    }

    void can_receive_callback(
        const Spec::Can& can,
        const View::Can& data
    ) override {

        // ============================================================
        // 临时 CAN 诊断代码
        // ============================================================
        //
        // 这一段现在不是最终的电机控制代码。
        //
        // 它只负责：
        //
        // C620
        //   ↓ CAN 反馈
        // CBoard
        //   ↓
        // can_receive_callback()
        //   ↓
        // 找到 DJI 电机反馈
        //   ↓
        // 打印 CAN 通道 + CAN ID
        //
        //
        // 我们现在还不知道朋友设置的 C620 ID，
        // 所以不能提前把反馈 ID 写死成 0x201。
        //
        // DJI 电机反馈常见 ID 范围：
        //
        // 0x201 ~ 0x208
        //
        // 例如：
        //
        // C620 ID 1
        //     ↓
        // CAN ID 0x201
        //
        // C620 ID 3
        //     ↓
        // CAN ID 0x203

        if (motor_feedback_found_) {
            // 已经成功找到过一次反馈以后，
            // 后续直接返回。
            //
            // C620 会持续、高频地发送电机反馈，
            // 如果每一帧都 RCLCPP_INFO，
            // 终端会被大量日志刷满。
            return;
        }

        if (
            data.is_extended_can_id ||
            data.is_remote_transmission ||
            data.can_data.size() < 8
        ) {
            // DJI 电机正常反馈使用标准 CAN 数据帧，
            // 并且反馈数据长度为 8 Byte。
            //
            // 如果：
            // - 是扩展 CAN ID
            // - 是 Remote Frame
            // - 数据不足 8 Byte
            //
            // 就不是我们当前想找的正常 DJI 电机反馈，
            // 直接忽略。
            return;
        }

        if (data.can_id < 0x201 || data.can_id > 0x208) {
            // 现在只关心 DJI 电机反馈 ID。
            //
            // 其他 CAN 设备可能也会往总线上发数据，
            // 这里先全部过滤掉，
            // 避免影响我们判断 C620 ID。
            return;
        }

        const char* can_name = "Unknown";

        // CBoard 源码已经确认：
        //
        // 物理 CAN1 → Spec::kCans.kCan1
        // 物理 CAN2 → Spec::kCans.kCan2
        //
        // 你朋友现在把 C620 接在物理 CAN1，
        // 因此正常情况下后面应该检测到 kCan1。

        if (can == Spec::kCans.kCan1) {
            can_name = "kCan1";
        } else if (can == Spec::kCans.kCan2) {
            can_name = "kCan2";
        }

        RCLCPP_INFO(
            get_logger(),
            "DJI motor feedback detected: %s, CAN ID = 0x%03X",
            can_name,
            static_cast<unsigned int>(data.can_id)
        );

        // 如果以后输出例如：
        //
        // DJI motor feedback detected:
        // kCan1, CAN ID = 0x203
        //
        // 那么我们就能同时知道：
        //
        // 物理连接：
        // CAN1 → kCan1
        //
        // C620 ID：
        // 0x203 → ID 3

        motor_feedback_found_ = true;

        // 标记：
        // “我们已经成功发现过一次 DJI 电机反馈。”
        //
        // 后续 CAN 帧就不再重复打印。
    }

private:
    bool motor_feedback_found_ = false;

    // 临时 CAN 诊断标志。
    //
    // false：
    // --------------------------------
    // 还没有检测到 DJI 电机反馈。
    //
    // true：
    // --------------------------------
    // 已经检测并打印过一次反馈，
    // 后续不再重复打印。
    //
    // 等我们真正确认 C620 ID 后，
    // 这一套临时诊断代码会被删除，
    // 换成真正的：
    //
    // motor_.store_status(...)
    //
    // 数据链。


    std::unique_ptr<librmcs::board::CBoard> board_;

    // CBoard 通信对象。
    //
    // 实际链路：
    //
    // 电脑
    //  ↓ USB
    // CBoard
    //  ↓
    // CAN / UART / DBUS
    //  ↓
    // 当前 Dr16MotorControl
    //
    //
    // 如果没有 board_：
    //
    // 虽然我们继承了 CBoard::Callback，
    // 但是实际上没有任何 CBoard 对象和电脑通信，
    // 回调函数也不会收到真实硬件数据。


    device::Dr16 dr16_;

    // DR16 遥控器状态对象。
    //
    // 完整的数据链是：
    //
    // DR16 遥控器
    //      ↓ 无线
    // DR16 接收机
    //      ↓ DBUS
    // CBoard
    //      ↓ USB
    // librmcs
    //      ↓
    // uart_receive_callback()
    //      ↓
    // dr16_.store_status()
    //      ↓
    // update()
    //      ↓
    // dr16_.update_status()
    //      ↓
    // joystick_left()
    // joystick_right()
    //
    // 后面我们会从这里读取摇杆值，
    // 再映射成 M3508 的目标速度。
};

} // namespace rmcs_core::hardware


#include <pluginlib/class_list_macros.hpp>
// pluginlib 的组件导出宏。
// 前面的里程碑已经验证过这一部分可以正常工作。

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::Dr16MotorControl,
    rmcs_executor::Component
)

// 把 Dr16MotorControl 导出成 RMCS Component 插件。
//
// plugins.xml 中也已经注册：
//
// rmcs_core::hardware::Dr16MotorControl
//
// 如果缺少这里的 EXPORT：
// 即使代码已经成功编译进 librmcs_core.so，
// RMCS Executor 也无法通过 pluginlib 创建这个类。