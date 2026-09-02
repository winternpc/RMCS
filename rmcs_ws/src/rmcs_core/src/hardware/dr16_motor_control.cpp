#include <memory>
// std::unique_ptr 和 std::make_unique 的定义。
// 后面我们用它保存 RmcsBoardLite 对象。

#include <rclcpp/node.hpp>
// ROS2 的 Node 定义。
// 我们后面会从 YAML / ROS2 参数中读取：
// 开发板序列号、电机参数、PID 参数等。

#include <rmcs_executor/component.hpp>
// RMCS 的 Component 基类。
// 只有继承它，当前类才能被 RMCS Executor 当成一个组件运行。

#include "hardware/device/dr16.hpp"
// RMCS 已经写好的 DR16 遥控器解析类。
// 后面 UART 收到 DR16 原始数据以后，
// 会交给这个对象保存和解析。

#include "librmcs/board/rmcs_board_lite.hpp"
// RMCS Board Lite 的接口。
// 它负责和真正的开发板通信，接收 UART / CAN 等数据。

namespace rmcs_core::hardware {

// 这是我们针对任务二新写的 hardware。
//
// 当前阶段只建立：
// 1. RMCS Component
// 2. ROS2 Node
// 3. Board Callback
// 4. RmcsBoardLite
// 5. DR16
//
// 暂时还没有接电机、滤波器和 PID。
class Dr16MotorControl
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::RmcsBoardLite::Callback {
public:
    Dr16MotorControl()
        : Node{
              get_component_name(),
              // 使用 RMCS Component 的名字作为 ROS2 Node 名字。

              rclcpp::NodeOptions{}
                  .automatically_declare_parameters_from_overrides(true)
              // 允许 YAML 中传入的参数自动声明。
              //
              // 后面我们就可以直接通过：
              // get_parameter("参数名")
              // 读取 YAML 中传进来的数据。
          } {

        board_ = std::make_unique<librmcs::board::RmcsBoardLite>(
            *this,
            get_parameter("board_serial").as_string()
        );

        // ↑ 创建真正负责与 RMCS 开发板通信的 Board 对象。
        //
        // 第一个参数 *this：
        // --------------------------------
        // 把当前 Dr16MotorControl 自己作为 Callback 对象交给 Board。
        //
        // 可以理解成：
        //
        // Board 收到 UART / CAN 数据
        //          ↓
        // 找 Callback
        //          ↓
        // 找到当前 Dr16MotorControl
        //          ↓
        // 调用我们以后写的回调函数
        //
        //
        // 第二个参数 board_serial：
        // --------------------------------
        // 从 YAML / ROS2 参数中读取开发板序列号。
        //
        // 一台电脑有可能连接多块 RMCS Board，
        // 所以需要通过序列号告诉程序：
        //
        // “这个 hardware 到底使用哪块开发板？”
    }

    void update() override {
        dr16_.update_status();
        // 把 uart_receive_callback() 之前保存的 DR16 原始数据，
        // 解析成摇杆、拨杆、鼠标、键盘等实际状态。
        //
        // RMCS Executor 会周期调用 update()，
        // 因此 DR16 状态也会周期刷新。
    }
    void uart_receive_callback(
    const Spec::Uart& uart,
    const View::Uart& data
) override {
    // 一块 RMCS Board 上不只有一个 UART，
    // 所以首先判断：
    // “这次收到的数据是不是来自 DR16 使用的 DBUS 串口？”

    if (uart == Spec::kUarts.kDbus) {
        dr16_.store_status(
            data.uart_data.data(),
            data.uart_data.size()
        );

        // uart_data.data()
        // → UART 这一帧数据在内存中的起始位置。
        //
        // uart_data.size()
        // → 这一帧一共有多少字节。
        //
        // store_status()
        // → 把原始 DR16 数据先保存下来。
        //
        // 后面的 dr16_.update_status()
        // 才会真正解析摇杆等状态。
    }
}
private:
    std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    // RMCS 开发板通信对象。
    //
    // 它负责：
    // USB / Board
    //      ↓
    // UART / CAN 数据
    //      ↓
    // 回调到当前 hardware
    //
    // 如果没有 board_：
    // 我们虽然继承了 Callback，
    // 但实际上根本没有 Board 对象给我们发送数据。


    device::Dr16 dr16_;
    // DR16 遥控器状态对象。
    //
    // 后面数据链会变成：
    //
    // DR16 接收机
    //      ↓
    // 开发板 UART / DBUS
    //      ↓
    // RmcsBoardLite
    //      ↓
    // uart_receive_callback()
    //      ↓
    // dr16_.store_status()
    //      ↓
    // dr16_.update_status()
    //      ↓
    // joystick_left()
    // joystick_right()
    //
    // 当前阶段只是先把这个对象创建出来。
};

} // namespace rmcs_core::hardware


#include <pluginlib/class_list_macros.hpp>
// pluginlib 的组件导出宏。
// 前面的里程碑已经验证过这部分可以正常工作。

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::Dr16MotorControl,
    rmcs_executor::Component
)
// 把 Dr16MotorControl 导出成 RMCS Component 插件。
//
// 如果没有这一句：
// 即使代码已经被编译进 librmcs_core.so，
// RMCS 的 pluginlib 也找不到并创建这个类。
