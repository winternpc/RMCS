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
// 第二周任务二：DR16 + M3508 速度闭环控制 Hardware
// ============================================================
//
// 当前已经实现的数据链：
//
// DR16 左摇杆 Y
//      ↓
// target_velocity_
//
//
// M3508
//      ↓
// motor_.velocity()
//      ↓
// raw_velocity_
//      ↓
// LowPassFilter
//      ↓
// filtered_velocity_
//
//
// 本阶段新增：
//
// target_velocity_ - filtered_velocity_
//      ↓
// velocity_error_
//      ↓
// PidCalculator
//      ↓
// control_torque_
//
//
// 注意：
//
// 当前 control_torque_ 只会被“计算和打印”，
// 还不会发送给 C620。
//
// 所以当前版本仍然不会主动驱动 M3508。
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
        // 1. 读取最大目标速度
        // ========================================================
        max_target_velocity_ =
            get_parameter("max_target_velocity").as_double();

        // 当前：
        //
        // max_target_velocity = 10 rad/s
        //
        // 所以左摇杆：
        //
        // +1 → +10 rad/s
        //  0 →   0 rad/s
        // -1 → -10 rad/s


        // ========================================================
        // 2. 读取低通滤波器截止频率
        // ========================================================
        velocity_filter_cutoff_hz_ =
            get_parameter("velocity_filter_cutoff_hz").as_double();


        // ========================================================
        // 3. 配置速度低通滤波器
        // ========================================================
        velocity_filter_.set_cutoff(
            velocity_filter_cutoff_hz_,
            kUpdateFrequencyHz
        );

        // 当前：
        //
        // 截止频率 = 10 Hz
        // 采样频率 = 1000 Hz


        // ========================================================
        // 4. 创建速度 PID
        // ========================================================
        velocity_pid_ =
            controller::pid::make_pid_calculator(
                *this,
                "velocity_"
            );

        // "velocity_" 是参数前缀。
        //
        // 所以 make_pid_calculator() 会寻找：
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
        //
        //
        // 这些参数我们全部放进 YAML。
        //
        //
        // 当前第一轮只使用：
        //
        // P
        //
        // 即：
        //
        // ki = 0
        // kd = 0
        //
        // 这样先把最基本的控制方向验证清楚。


        // ========================================================
        // 5. 配置 M3508
        // ========================================================
        motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kM3508,
                3
            }
        );

        // 已经实机确认：
        //
        // C620 ID = 3
        //
        // M3508 反馈：
        //
        // CAN1
        // CAN ID = 0x203


        // ========================================================
        // 6. 创建 CBoard
        // ========================================================
        board_ = std::make_unique<librmcs::board::CBoard>(
            *this,
            get_parameter("board_serial").as_string()
        );
    }


    // ============================================================
    // RMCS 1000 Hz 周期更新函数
    // ============================================================
    void update() override {

        // ========================================================
        // 一、更新 DR16
        // ========================================================
        dr16_.update_status();


        // ========================================================
        // 二、DR16 左摇杆 → 目标速度
        // ========================================================
        if (dr16_.valid()) {

            target_velocity_ =
                dr16_.joystick_left().y()
                * max_target_velocity_;

        } else {

            // 遥控器掉线：
            //
            // 目标速度必须立即清零。
            target_velocity_ = 0.0;
        }


        // ========================================================
        // 三、更新 M3508 状态
        // ========================================================
        motor_.update_status();


        // ========================================================
        // 四、原始速度 → 低通滤波
        // ========================================================
        if (motor_feedback_found_) {

            raw_velocity_ =
                motor_.velocity();

            filtered_velocity_ =
                velocity_filter_.update(
                    raw_velocity_
                );

        } else {

            // 还没有收到真实电机反馈。
            raw_velocity_ = 0.0;
            filtered_velocity_ = 0.0;

            // 清除滤波器历史。
            velocity_filter_.reset();
        }


        // ========================================================
        // 五、计算速度 PID
        // ========================================================
        //
        // 只有两个条件同时满足时才允许 PID 工作：
        //
        // 1. DR16 在线
        // 2. M3508 有真实反馈
        //
        // 否则：
        //
        // control_torque = 0
        //
        // 并清空 PID 内部状态。
        if (
            dr16_.valid()
            && motor_feedback_found_
        ) {

            // ----------------------------------------------------
            // 1. 计算速度误差
            // ----------------------------------------------------
            velocity_error_ =
                target_velocity_
                - filtered_velocity_;

            // 这是整个闭环最重要的一行之一：
            //
            // error = target - actual
            //
            //
            // 例子：
            //
            // target = 10
            // actual = 6
            //
            // error = 4
            //
            // → 说明电机还太慢
            //
            //
            // target = 5
            // actual = 8
            //
            // error = -3
            //
            // → 说明电机太快了


            // ----------------------------------------------------
            // 2. 把误差交给 PID
            // ----------------------------------------------------
            control_torque_ =
                velocity_pid_.update(
                    velocity_error_
                );

            // 当前：
            //
            // kp = 0.05
            // ki = 0
            // kd = 0
            //
            // 所以暂时可以近似理解成：
            //
            // control_torque
            // =
            // 0.05 × error
            //
            //
            // 例如：
            //
            // error = 10 rad/s
            //
            // control_torque
            // = 0.05 × 10
            // = 0.5
            //
            //
            // 但我们 YAML 还会把输出限制在：
            //
            // [-0.5, +0.5]
            //
            // 防止后面实机初次控制时输出过大。

        } else {

            // ====================================================
            // 安全状态
            // ====================================================
            velocity_error_ = 0.0;
            control_torque_ = 0.0;

            // 清掉 PID 里面的：
            //
            // last_err_
            // err_integral_
            //
            // 避免设备重新连接以后，
            // PID 带着旧状态继续工作。
            velocity_pid_.reset();
        }


        // ========================================================
        // 六、每秒打印一次完整控制链
        // ========================================================
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
                "PID output=%.3f",

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

            // 后面可能看到：
            //
            // joy=0.500
            // target=5.000
            // filtered=0.000
            //
            // error：
            //
            // 5 - 0
            // = 5
            //
            //
            // kp=0.05：
            //
            // PID output：
            //
            // 5 × 0.05
            // = 0.25
            //
            //
            // 这就是我们本阶段要验证的东西。
        }


        // ========================================================
        // 注意！
        // ========================================================
        //
        // 当前这里没有：
        //
        // motor_.generate_command(...)
        //
        // 也没有：
        //
        // CAN transmit
        //
        // 所以 control_torque_ 现在只是一个计算结果。
        //
        // 电机不会因为 PID output 变化而主动转动。
    }


    // ============================================================
    // DR16 DBUS 接收
    // ============================================================
    void uart_receive_callback(
        const Spec::Uart& uart,
        const View::Uart& data
    ) override {

        if (uart != Spec::kUarts.kDbus) {
            return;
        }


        // 第一次收到 DBUS 时打印一次。
        if (!dbus_packet_found_) {

            RCLCPP_INFO(
                get_logger(),
                "DBUS packet received, size = %zu bytes",
                data.uart_data.size()
            );

            dbus_packet_found_ = true;
        }


        // 保存原始 DR16 数据。
        dr16_.store_status(
            data.uart_data.data(),
            data.uart_data.size()
        );
    }


    // ============================================================
    // M3508 CAN 接收
    // ============================================================
    void can_receive_callback(
        const Spec::Can& can,
        const View::Can& data
    ) override {

        // 只使用 CAN1。
        if (can != Spec::kCans.kCan1) {
            return;
        }


        // DJI 电机正常反馈：
        //
        // 标准 CAN ID
        // 非 Remote Frame
        // 8 Byte
        if (
            data.is_extended_can_id ||
            data.is_remote_transmission ||
            data.can_data.size() != 8
        ) {
            return;
        }


        // 让 DjiMotor 判断是不是：
        //
        // M3508 ID = 3
        //
        // 即：
        //
        // CAN ID = 0x203
        const bool matched =
            motor_.match_then_store_status(
                data.can_id,
                data.can_data
            );


        // 第一次收到真实反馈时提示。
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

            // 新连接时清空历史状态。
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
    // 目标速度
    // ============================================================
    double target_velocity_ = 0.0;


    // ============================================================
    // 低通滤波截止频率
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

    // error
    // =
    // target_velocity_
    // -
    // filtered_velocity_


    // ============================================================
    // PID 计算结果
    // ============================================================
    double control_torque_ = 0.0;

    // 目前只是计算。
    //
    // 还没有发送给电机。


    // ============================================================
    // 一阶低通滤波器
    // ============================================================
    filter::LowPassFilter<1>
        velocity_filter_{1.0};


    // ============================================================
    // 速度 PID
    // ============================================================
    controller::pid::PidCalculator
        velocity_pid_{};

    // 当前 PID：
    //
    // 输入：
    //
    // velocity_error_
    //
    // 输出：
    //
    // control_torque_


    // ============================================================
    // DBUS 是否收到过数据
    // ============================================================
    bool dbus_packet_found_ = false;


    // ============================================================
    // 电机是否收到过反馈
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

    // 当前仍然只读取反馈。
    //
    // 还没有生成并发送控制 CAN 帧。
};

} // namespace rmcs_core::hardware


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::Dr16MotorControl,
    rmcs_executor::Component
)