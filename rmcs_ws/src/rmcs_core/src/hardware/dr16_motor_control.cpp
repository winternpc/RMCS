#include <cstdint>
#include <memory>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "filter/low_pass_filter.hpp"

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"

#include "librmcs/board/c_board.hpp"


namespace rmcs_core::hardware {

// ============================================================
// 第二周任务二：DR16 + M3508 速度控制 Hardware
// ============================================================
//
// 当前数据链：
//
// DR16 左摇杆 Y
//      ↓
// target_velocity_
//
//
// M3508
//      ↓ CAN1 / 0x203
// motor_.velocity()
//      ↓
// raw_velocity_
//      ↓
// LowPassFilter
//      ↓
// filtered_velocity_
//
//
// 下一阶段才会继续：
//
// target_velocity_ - filtered_velocity_
//      ↓
// error
//      ↓
// PID
//      ↓
// control_torque
//      ↓
// CAN
//      ↓
// M3508
//
//
// 当前版本仍然不会主动发送电机控制命令。
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
        // 1. 读取 DR16 最大目标速度
        // ========================================================
        max_target_velocity_ =
            get_parameter("max_target_velocity").as_double();

        // 当前 YAML：
        //
        // max_target_velocity: 10.0
        //
        // 所以：
        //
        // 左摇杆 +1 → +10 rad/s
        // 左摇杆  0 →   0 rad/s
        // 左摇杆 -1 → -10 rad/s


        // ========================================================
        // 2. 读取低通滤波器截止频率
        // ========================================================
        velocity_filter_cutoff_hz_ =
            get_parameter("velocity_filter_cutoff_hz").as_double();

        // 截止频率：
        //
        // 可以先简单理解成：
        //
        // “变化多快的数据允许比较完整地通过”
        //
        // 截止频率越低：
        // → 越平滑
        // → 但是反应越慢
        //
        // 截止频率越高：
        // → 反应越快
        // → 但是滤波效果越弱


        // ========================================================
        // 3. 配置低通滤波器
        // ========================================================
        velocity_filter_.set_cutoff(
            velocity_filter_cutoff_hz_,
            kUpdateFrequencyHz
        );

        // LowPassFilter 源码中的参数顺序：
        //
        // set_cutoff(
        //     cutoff_frequency,
        //     sampling_frequency
        // )
        //
        // 我们现在：
        //
        // cutoff   = 10 Hz
        // sampling = 1000 Hz
        //
        // sampling = 1000 Hz 的原因：
        //
        // rmcs_executor:
        //   update_rate: 1000.0
        //
        // 即 update() 大约每 1 ms 执行一次。


        // ========================================================
        // 4. 配置 M3508
        // ========================================================
        motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kM3508,
                3
            }
        );

        // 实机已经确认：
        //
        // M3508
        // C620 ID = 3
        //
        // 反馈：
        //
        // CAN1
        // CAN ID = 0x203


        // ========================================================
        // 5. 创建 CBoard
        // ========================================================
        board_ = std::make_unique<librmcs::board::CBoard>(
            *this,
            get_parameter("board_serial").as_string()
        );
    }


    // ============================================================
    // RMCS 周期更新
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
            // 目标速度立即清零。
            target_velocity_ = 0.0;
        }


        // ========================================================
        // 三、更新 M3508 状态
        // ========================================================
        motor_.update_status();

        // can_receive_callback()
        // 会保存原始 CAN 数据。
        //
        // update_status()
        // 再把 CAN 数据解析成：
        //
        // angle()
        // velocity()
        // torque()
        // temperature()


        // ========================================================
        // 四、M3508 原始速度 → LowPassFilter
        // ========================================================
        if (motor_feedback_found_) {

            // --------------------------------------------
            // 1. 取得原始电机速度
            // --------------------------------------------
            raw_velocity_ =
                motor_.velocity();

            // 单位：
            //
            // rad/s


            // --------------------------------------------
            // 2. 送进一阶低通滤波器
            // --------------------------------------------
            filtered_velocity_ =
                velocity_filter_.update(
                    raw_velocity_
                );

            // LowPassFilter 内部核心公式：
            //
            // output
            // =
            // alpha × input
            // +
            // (1 - alpha) × previous_output
            //
            //
            // 可以理解成：
            //
            // 新结果
            // =
            // 一部分“最新数据”
            // +
            // 一部分“上一时刻数据”
            //
            // 所以速度不会因为单个瞬时抖动
            // 立刻发生很大的跳变。

        } else {

            // 还没有真实电机反馈。
            //
            // 这时候不能把 motor_.velocity()
            // 当成有效测量值。
            raw_velocity_ = 0.0;
            filtered_velocity_ = 0.0;

            // 清掉滤波器的历史状态。
            velocity_filter_.reset();
        }


        // ========================================================
        // 五、每秒打印一次状态
        // ========================================================
        ++status_log_counter_;

        if (status_log_counter_ >= 1000) {

            status_log_counter_ = 0;

            RCLCPP_INFO(
                get_logger(),

                "DR16=%s, "
                "joystick=%.3f, "
                "target=%.3f rad/s, "
                "raw=%.3f rad/s, "
                "filtered=%.3f rad/s",

                dr16_.valid()
                    ? "valid"
                    : "invalid",

                dr16_.joystick_left().y(),

                target_velocity_,

                raw_velocity_,

                filtered_velocity_
            );

            // 后面会看到类似：
            //
            // DR16=valid,
            // joystick=0.500,
            // target=5.000 rad/s,
            // raw=4.800 rad/s,
            // filtered=4.600 rad/s
            //
            //
            // target：
            // 想让电机达到多少速度
            //
            // raw：
            // M3508 当前原始回传速度
            //
            // filtered：
            // 原始速度经过低通滤波之后的速度
        }
    }


    // ============================================================
    // DR16 / DBUS 接收回调
    // ============================================================
    void uart_receive_callback(
        const Spec::Uart& uart,
        const View::Uart& data
    ) override {

        // DR16 使用 CBoard 的 DBUS。
        if (uart != Spec::kUarts.kDbus) {
            return;
        }


        // ========================================================
        // 第一次收到 DBUS 时打印一次
        // ========================================================
        if (!dbus_packet_found_) {

            RCLCPP_INFO(
                get_logger(),
                "DBUS packet received, size = %zu bytes",
                data.uart_data.size()
            );

            dbus_packet_found_ = true;
        }


        // ========================================================
        // 保存 DR16 原始数据
        // ========================================================
        dr16_.store_status(
            data.uart_data.data(),
            data.uart_data.size()
        );
    }


    // ============================================================
    // M3508 CAN 接收回调
    // ============================================================
    void can_receive_callback(
        const Spec::Can& can,
        const View::Can& data
    ) override {

        // ========================================================
        // 1. 只接收 CAN1
        // ========================================================
        if (can != Spec::kCans.kCan1) {
            return;
        }


        // ========================================================
        // 2. 检查 CAN 帧
        // ========================================================
        if (
            data.is_extended_can_id ||
            data.is_remote_transmission ||
            data.can_data.size() != 8
        ) {
            return;
        }


        // ========================================================
        // 3. 判断是不是当前 M3508
        // ========================================================
        const bool matched =
            motor_.match_then_store_status(
                data.can_id,
                data.can_data
            );

        // 当前：
        //
        // M3508 ID = 3
        //
        // 所以反馈 CAN ID：
        //
        // 0x200 + 3
        // =
        // 0x203


        // ========================================================
        // 4. 第一次收到正确反馈时提示
        // ========================================================
        if (matched && !motor_feedback_found_) {

            RCLCPP_INFO(
                get_logger(),
                "M3508 feedback connected: CAN1, ID = 0x%03X",
                static_cast<unsigned int>(
                    data.can_id
                )
            );

            motor_feedback_found_ = true;

            // 新连接电机反馈时，
            // 清空之前滤波器的历史数据。
            velocity_filter_.reset();
        }
    }


private:

    // ============================================================
    // 当前 RMCS 更新频率
    // ============================================================
    static constexpr double
        kUpdateFrequencyHz = 1000.0;

    // 当前 YAML：
    //
    // update_rate: 1000.0
    //
    // 所以这里保持一致。
    //
    // 如果以后改 RMCS update_rate，
    // 这里的采样频率也必须一起修改。


    // ============================================================
    // DR16 最大目标速度
    // ============================================================
    double max_target_velocity_ = 0.0;

    // 单位：
    //
    // rad/s


    // ============================================================
    // 当前目标速度
    // ============================================================
    double target_velocity_ = 0.0;

    // 来源：
    //
    // DR16 左摇杆 Y
    // ×
    // max_target_velocity_


    // ============================================================
    // 低通滤波器截止频率
    // ============================================================
    double velocity_filter_cutoff_hz_ = 0.0;

    // 单位：
    //
    // Hz
    //
    // 当前先使用：
    //
    // 10 Hz


    // ============================================================
    // 原始 M3508 速度
    // ============================================================
    double raw_velocity_ = 0.0;

    // 来源：
    //
    // motor_.velocity()
    //
    // 单位：
    //
    // rad/s


    // ============================================================
    // 滤波后的 M3508 速度
    // ============================================================
    double filtered_velocity_ = 0.0;

    // 后面 PID 不直接使用：
    //
    // motor_.velocity()
    //
    // 而是使用：
    //
    // filtered_velocity_


    // ============================================================
    // 一阶低通滤波器
    // ============================================================
    filter::LowPassFilter<1>
        velocity_filter_{1.0};

    // <1>
    //
    // 表示：
    //
    // 这个滤波器处理一个 double。
    //
    // 我们处理的是：
    //
    // M3508 velocity
    //
    // 所以使用 LowPassFilter<1>。


    // ============================================================
    // 是否收到 DBUS
    // ============================================================
    bool dbus_packet_found_ = false;


    // ============================================================
    // 是否收到 M3508 反馈
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

    // 当前依然只是：
    //
    // 接收 M3508 反馈。
    //
    // 还没有调用：
    //
    // generate_command()
    // can_transmit()
    //
    // 所以不会主动驱动电机。
};

} // namespace rmcs_core::hardware


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::Dr16MotorControl,
    rmcs_executor::Component
)