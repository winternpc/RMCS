#include <cmath>
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
// DR16 控制 M3508 速度闭环
// ============================================================
//
// 整体控制流程：
//
// DR16 左摇杆 Y
//        ↓
// target_velocity_ 目标速度
//        ↓
// 和电机真实速度进行比较
//        ↓
// velocity_error_ = target - actual
//        ↓
// PI 控制器
//        ↓
// control_torque_
//        ↓
// DjiMotor::generate_command()
//        ↓
// CAN1 / 0x200
//        ↓
// C620 ID = 3
//        ↓
// M3508
//
// 电机反馈则反过来：
//
// M3508
//        ↓
// C620
//        ↓
// CAN ID = 0x203
//        ↓
// motor_.velocity()
//        ↓
// LowPassFilter
//        ↓
// filtered_velocity_
//        ↓
// 再送回 PI 控制器
//
// 这样就形成了一个真正的速度闭环。
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
        // 1. 读取最大目标速度
        // --------------------------------------------------------
        //
        // YAML 中：
        //
        // max_target_velocity: 10.0
        //
        // 表示摇杆推满时，
        // 目标速度最大为 10 rad/s。
        //
        max_target_velocity_ =
            get_parameter("max_target_velocity").as_double();


        // --------------------------------------------------------
        // 2. 读取低通滤波截止频率
        // --------------------------------------------------------
        //
        // 电机速度反馈并不是完全平滑的，
        // 会有一些小幅噪声和跳动。
        //
        // 所以不能直接把原始速度全部交给 PID。
        //
        velocity_filter_cutoff_hz_ =
            get_parameter("velocity_filter_cutoff_hz").as_double();


        // --------------------------------------------------------
        // 3. 配置速度低通滤波器
        // --------------------------------------------------------
        //
        // set_cutoff() 需要两个参数：
        //
        // 第一个：
        // 截止频率
        //
        // 第二个：
        // 采样频率
        //
        // 当前 RMCS update_rate = 1000 Hz，
        // 所以控制周期约为 1 ms。
        //
        velocity_filter_.set_cutoff(
            velocity_filter_cutoff_hz_,
            kUpdateFrequencyHz
        );


        // --------------------------------------------------------
        // 4. 创建速度 PID
        // --------------------------------------------------------
        //
        // "velocity_" 是参数前缀。
        //
        // 所以它会自动从 YAML 中读取：
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
        velocity_pid_ =
            controller::pid::make_pid_calculator(
                *this,
                "velocity_"
            );


        // --------------------------------------------------------
        // 5. 配置 M3508
        // --------------------------------------------------------
        //
        // 当前实机使用的是：
        //
        // DJI M3508
        // +
        // C620
        //
        // C620 ID 设置为 3。
        //
        motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kM3508,
                3
            }
        );

        // 对于 ID = 3：
        //
        // 反馈 CAN ID：
        //
        // 0x200 + 3 = 0x203
        //
        // 控制帧：
        //
        // CAN ID = 0x200
        //
        // 控制值放在第三个 Quarter。


        // --------------------------------------------------------
        // 6. 创建 CBoard
        // --------------------------------------------------------
        //
        // CBoard 负责：
        //
        // CAN 接收
        // CAN 发送
        // DBUS 接收
        //
        board_ =
            std::make_unique<librmcs::board::CBoard>(
                *this,
                get_parameter("board_serial").as_string()
            );
    }


    // ============================================================
    // RMCS 主循环
    // ============================================================
    //
    // YAML 中 update_rate = 1000，
    // 所以这个函数正常情况下每秒执行约 1000 次。
    //
    void update() override {

        // --------------------------------------------------------
        // 1. 更新 DR16 状态
        // --------------------------------------------------------
        //
        // uart_receive_callback() 负责保存最新 DBUS 数据。
        //
        // update_status() 则负责把原始数据解析成：
        //
        // 摇杆
        // 拨杆
        // 键鼠
        // valid 状态
        //
        dr16_.update_status();


        // --------------------------------------------------------
        // 2. 左摇杆 Y → 目标速度
        // --------------------------------------------------------
        if (dr16_.valid()) {

            // joystick_left().y()
            //
            // 大致范围：
            //
            // -1.0 ~ +1.0
            //
            const double joystick_y =
                dr16_.joystick_left().y();


            // ----------------------------------------------------
            // 摇杆死区
            // ----------------------------------------------------
            //
            // 实际遥控器回中以后，
            // 数值不一定永远严格等于 0。
            //
            // 例如可能出现：
            //
            // 0.01
            // -0.02
            // 0.03
            //
            // 如果直接映射，
            // 电机可能一直收到一个很小的目标速度。
            //
            // 所以规定：
            //
            // |joystick| < 0.05
            //
            // 就直接认为摇杆已经回中。
            //
            if (std::abs(joystick_y) < kJoystickDeadzone) {

                target_velocity_ = 0.0;

            } else {

                // 摇杆比例 × 最大速度
                //
                // 例如：
                //
                // joystick = 0.5
                // max_target_velocity = 10
                //
                // target = 5 rad/s
                //
                target_velocity_ =
                    joystick_y
                    * max_target_velocity_;
            }

        } else {

            // ----------------------------------------------------
            // DR16 掉线保护
            // ----------------------------------------------------
            //
            // 如果遥控器已经失效，
            // 不允许继续保持原来的目标速度。
            //
            target_velocity_ = 0.0;
        }


        // --------------------------------------------------------
        // 3. 计算电机反馈“年龄”
        // --------------------------------------------------------
        //
        // 之前我们只用：
        //
        // motor_feedback_found_
        //
        // 它只能说明：
        //
        // “曾经收到过一次反馈”
        //
        // 但不能说明：
        //
        // “现在仍然持续收到反馈”
        //
        // 这会产生一个问题：
        //
        // 假设最后一次速度是 2.165 rad/s，
        // 然后 CAN 反馈突然断掉，
        //
        // 如果没有超时判断，
        // 程序可能一直把 2.165 当作最新速度。
        //
        // 所以这里使用一个计数器：
        //
        // 每执行一个控制周期 +1，
        // 每收到一次真正的新 0x203 反馈重新清零。
        //
        if (
            motor_feedback_found_
            && motor_feedback_age_cycles_
                <= kMotorFeedbackTimeoutCycles
        ) {
            ++motor_feedback_age_cycles_;
        }


        // --------------------------------------------------------
        // 4. 判断当前反馈是否仍然有效
        // --------------------------------------------------------
        //
        // 条件：
        //
        // ① 曾经收到过 M3508 反馈
        // ② 最近 100 ms 内仍然收到过新反馈
        //
        const bool motor_feedback_alive =
            motor_feedback_found_
            && motor_feedback_age_cycles_
                <= kMotorFeedbackTimeoutCycles;


        // --------------------------------------------------------
        // 5. 更新 DjiMotor 内部状态
        // --------------------------------------------------------
        //
        // can_receive_callback() 负责保存原始 CAN 数据。
        //
        // update_status() 会把原始数据解析成：
        //
        // angle
        // velocity
        // torque
        // 等物理量。
        //
        motor_.update_status();


        // --------------------------------------------------------
        // 6. 读取速度并低通滤波
        // --------------------------------------------------------
        if (motor_feedback_alive) {

            // 原始速度
            raw_velocity_ =
                motor_.velocity();


            // 经过低通滤波后的速度
            filtered_velocity_ =
                velocity_filter_.update(
                    raw_velocity_
                );

        } else {

            // ----------------------------------------------------
            // 反馈已经失效
            // ----------------------------------------------------
            //
            // 不能继续使用最后一次旧速度。
            //
            raw_velocity_ = 0.0;
            filtered_velocity_ = 0.0;


            // 同时清掉滤波器历史数据。
            velocity_filter_.reset();
        }


        // --------------------------------------------------------
        // 7. PI 速度闭环
        // --------------------------------------------------------
        //
        // 只有：
        //
        // DR16 正常
        // &&
        // 电机反馈正常
        //
        // 才允许产生控制输出。
        //
        if (
            dr16_.valid()
            && motor_feedback_alive
        ) {

            // ----------------------------------------------------
            // 摇杆回中时清掉积分历史
            // ----------------------------------------------------
            //
            // 前面的实机测试发现：
            //
            // 加入 I 项后，
            // 即使 target 已经回到 0，
            // 积分仍然可能残留一个控制量。
            //
            // 例如：
            //
            // target = 0
            // error  = 0
            // torque_cmd = 0.05
            //
            // 所以当操作者明确松开摇杆后，
            // 把之前积累的积分清掉。
            //
            if (target_velocity_ == 0.0) {
                velocity_pid_.reset();
            }


            // ----------------------------------------------------
            // 计算速度误差
            // ----------------------------------------------------
            //
            // error = target - actual
            //
            // 例如：
            //
            // target = 5
            // actual = 3
            //
            // error = +2
            //
            // 说明电机还太慢。
            //
            // 如果：
            //
            // target = 5
            // actual = 6
            //
            // error = -1
            //
            // 说明电机已经跑快了。
            //
            velocity_error_ =
                target_velocity_
                - filtered_velocity_;


            // ----------------------------------------------------
            // PID / PI 计算
            // ----------------------------------------------------
            //
            // 当前参数实际上是：
            //
            // P ≠ 0
            // I ≠ 0
            // D = 0
            //
            // 所以目前本质上是 PI 控制。
            //
            // P：
            // 根据当前误差立即纠正。
            //
            // I：
            // 对长期存在的小误差进行补偿，
            // 主要帮助克服低速时的静摩擦和稳态误差。
            //
            control_torque_ =
                velocity_pid_.update(
                    velocity_error_
                );

        } else {

            // ----------------------------------------------------
            // 安全状态
            // ----------------------------------------------------
            //
            // 以下任意情况发生：
            //
            // DR16 掉线
            // 或
            // M3508 反馈超时
            //
            // 都直接停止控制。
            //
            velocity_error_ = 0.0;
            control_torque_ = 0.0;


            // 把 PID 内部历史也清掉，
            // 避免重新恢复以后继续使用旧积分。
            velocity_pid_.reset();
        }


        // --------------------------------------------------------
        // 8. CAN 发送
        // --------------------------------------------------------
        //
        // 无论正常还是异常，
        // 每个周期都会发送一次控制帧。
        //
        // 异常情况下：
        //
        // control_torque_ = 0
        //
        // 所以实际发送的就是 0 控制量。
        //
        command_update();


        // --------------------------------------------------------
        // 9. 状态日志
        // --------------------------------------------------------
        //
        // update() 是 1000 Hz，
        // 如果每个周期都打印日志，
        // 终端会完全刷屏。
        //
        // 所以每 1000 个周期打印一次，
        // 大约就是每秒一次。
        //
        ++status_log_counter_;

        if (status_log_counter_ >= 1000) {

            status_log_counter_ = 0;

            RCLCPP_INFO(
                get_logger(),

                "DR16=%s, "
                "feedback=%s, "
                "joy=%.3f, "
                "target=%.3f, "
                "raw=%.3f, "
                "filtered=%.3f, "
                "error=%.3f, "
                "torque_cmd=%.3f",

                dr16_.valid()
                    ? "valid"
                    : "invalid",

                motor_feedback_alive
                    ? "alive"
                    : "lost",

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
    // 向 C620 发送控制量
    // ============================================================
    void command_update() {

        // --------------------------------------------------------
        // start_transmit()
        // --------------------------------------------------------
        //
        // 这里直接仿照当前 RMCS 主线 flight.cpp。
        //
        // start_transmit()
        // 建立这一次 CBoard 发送过程。
        //
        auto builder =
            board_->start_transmit();


        // --------------------------------------------------------
        // CAN1 / 0x200
        // --------------------------------------------------------
        //
        // DJI C620 ID 1~4 共用控制帧：
        //
        // CAN ID = 0x200
        //
        // 8 个字节分成四组，
        // 每组两个字节。
        //
        // 第 1 组 → ID 1
        // 第 2 组 → ID 2
        // 第 3 组 → ID 3
        // 第 4 组 → ID 4
        //
        // 我们的 C620 ID = 3，
        // 所以控制量必须放第三组。
        //
        builder.can_transmit(
            Spec::kCans.kCan1,
            {
                .can_id = 0x200,

                .can_data =
                    device::CanPacket8{

                        // C620 ID 1，没有使用
                        device::CanPacket8::PaddingQuarter{},

                        // C620 ID 2，没有使用
                        device::CanPacket8::PaddingQuarter{},

                        // C620 ID 3，就是当前 M3508
                        //
                        // generate_command()
                        // 会把物理控制量转换成
                        // C620 能接受的原始命令值。
                        motor_.generate_command(
                            control_torque_
                        ),

                        // C620 ID 4，没有使用
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            }
        );
    }


    // ============================================================
    // DR16 / DBUS 接收回调
    // ============================================================
    void uart_receive_callback(
        const Spec::Uart& uart,
        const View::Uart& data
    ) override {

        // 当前遥控器只从 DBUS 接口读取。
        if (uart != Spec::kUarts.kDbus) {
            return;
        }


        // 第一次收到 DBUS 数据时打印一次，
        // 用来确认遥控器通信已经真正打通。
        if (!dbus_packet_found_) {

            RCLCPP_INFO(
                get_logger(),
                "DBUS packet received, size = %zu bytes",
                data.uart_data.size()
            );

            dbus_packet_found_ = true;
        }


        // 把 DBUS 原始数据交给 Dr16 保存。
        //
        // 真正的摇杆解析在 update_status() 中完成。
        //
        dr16_.store_status(
            data.uart_data.data(),
            data.uart_data.size()
        );
    }


    // ============================================================
    // M3508 CAN 反馈接收回调
    // ============================================================
    void can_receive_callback(
        const Spec::Can& can,
        const View::Can& data
    ) override {

        // 当前 M3508 接在 CBoard CAN1。
        if (can != Spec::kCans.kCan1) {
            return;
        }


        // DJI 电机正常反馈：
        //
        // 标准 CAN ID
        // 非 Remote Frame
        // 8 字节数据
        //
        if (
            data.is_extended_can_id
            || data.is_remote_transmission
            || data.can_data.size() != 8
        ) {
            return;
        }


        // --------------------------------------------------------
        // 判断是不是当前 ID=3 的 M3508
        // --------------------------------------------------------
        //
        // DjiMotor 已经知道自己的 ID 是 3。
        //
        // match_then_store_status() 会：
        //
        // ① 判断 CAN ID 是否匹配
        // ② 如果匹配就保存反馈
        //
        const bool matched =
            motor_.match_then_store_status(
                data.can_id,
                data.can_data
            );


        if (matched) {

            // ----------------------------------------------------
            // 收到新的反馈，反馈年龄清零
            // ----------------------------------------------------
            //
            // 这是反馈超时保护最关键的一行。
            //
            // 如果 CAN 一直正常，
            // 这个值会不断被重新变成 0。
            //
            // 如果 CAN 停止，
            // update() 就会不断让它增加，
            // 最终超过 100。
            //
            motor_feedback_age_cycles_ = 0;


            // ----------------------------------------------------
            // 第一次发现 M3508
            // ----------------------------------------------------
            if (!motor_feedback_found_) {

                RCLCPP_INFO(
                    get_logger(),
                    "M3508 feedback connected: CAN1, ID = 0x%03X",
                    static_cast<unsigned int>(
                        data.can_id
                    )
                );


                motor_feedback_found_ = true;


                // 第一次建立反馈时，
                // 不使用之前可能残留的滤波数据。
                velocity_filter_.reset();


                // PID 历史同样清零。
                velocity_pid_.reset();
            }
        }
    }


private:

    // ============================================================
    // 控制循环频率
    // ============================================================
    //
    // 当前 YAML：
    //
    // update_rate = 1000 Hz
    //
    static constexpr double
        kUpdateFrequencyHz = 1000.0;


    // ============================================================
    // DR16 摇杆死区
    // ============================================================
    //
    // ±5% 以内直接认为摇杆回中。
    //
    static constexpr double
        kJoystickDeadzone = 0.05;


    // ============================================================
    // 电机反馈超时
    // ============================================================
    //
    // 1000 Hz 下：
    //
    // 1 个周期 ≈ 1 ms
    //
    // 100 个周期 ≈ 100 ms
    //
    // 超过约 100 ms 没有收到新的 M3508 反馈，
    // 就认为反馈已经丢失。
    //
    static constexpr std::uint32_t
        kMotorFeedbackTimeoutCycles = 100;


    // 最大目标速度
    double max_target_velocity_ = 0.0;


    // 当前目标速度
    double target_velocity_ = 0.0;


    // 低通滤波截止频率
    double velocity_filter_cutoff_hz_ = 0.0;


    // M3508 原始速度
    double raw_velocity_ = 0.0;


    // 低通滤波之后的速度
    double filtered_velocity_ = 0.0;


    // target - actual
    double velocity_error_ = 0.0;


    // PI 最终计算出来的控制量
    double control_torque_ = 0.0;


    // ============================================================
    // 速度低通滤波器
    // ============================================================
    filter::LowPassFilter<1>
        velocity_filter_{1.0};


    // ============================================================
    // 速度 PID
    // ============================================================
    //
    // 当前：
    //
    // kp != 0
    // ki != 0
    // kd = 0
    //
    // 所以实际上使用的是 PI。
    //
    controller::pid::PidCalculator
        velocity_pid_{};


    // 是否已经收到过 DBUS 数据
    bool dbus_packet_found_ = false;


    // 是否曾经收到过 M3508 反馈
    bool motor_feedback_found_ = false;


    // ============================================================
    // 电机反馈年龄
    // ============================================================
    //
    // 初始化成超过超时值，
    // 防止程序刚启动时误认为反馈有效。
    //
    std::uint32_t motor_feedback_age_cycles_ =
        kMotorFeedbackTimeoutCycles + 1;


    // 日志计数器
    std::uint32_t status_log_counter_ = 0;


    // CBoard
    std::unique_ptr<librmcs::board::CBoard>
        board_;


    // DR16
    device::Dr16 dr16_;


    // ============================================================
    // M3508
    // ============================================================
    //
    // 第一个 *this：
    // 电机状态所属的 RMCS Component
    //
    // 第二个 *this：
    // 电机命令所属的 RMCS Component
    //
    // "/motor"：
    // 电机接口名称前缀
    //
    device::DjiMotor motor_{
        *this,
        *this,
        "/motor"
    };
};

} // namespace rmcs_core::hardware


// ============================================================
// 注册 RMCS Component
// ============================================================
//
// 没有这个宏，pluginlib 就无法通过配置文件
// 创建 Dr16MotorControl。
//
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::Dr16MotorControl,
    rmcs_executor::Component
)