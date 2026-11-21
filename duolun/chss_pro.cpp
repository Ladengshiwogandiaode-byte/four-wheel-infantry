//
// Created by ningj on 2026/11/9.
//
#include <librm.hpp>

#include <chrono>
#include <cmath>
#include "can.h"
#include "usart.h"
#include "math.h"
#include "referee.h"

extern "C" {
volatile uint32_t fm_chassis_rx_count = 0;
volatile uint32_t fm_chassis_online = 0;
volatile float fm_chassis_power_scale = 1.0f;
volatile uint32_t fm_super_cap_tx_count = 0;
volatile uint16_t fm_super_cap_tx_power_limit = 0;

volatile float fm_steer_raw_lf = 0.0f;
volatile float fm_steer_raw_rf = 0.0f;
volatile float fm_steer_raw_lb = 0.0f;
volatile float fm_steer_raw_rb = 0.0f;

volatile uint16_t fm_wheel_encoder_lf = 0;
volatile uint16_t fm_wheel_encoder_rf = 0;
volatile uint16_t fm_wheel_encoder_lb = 0;
volatile uint16_t fm_wheel_encoder_rb = 0;

volatile int16_t fm_wheel_rpm_lf = 0;
volatile int16_t fm_wheel_rpm_rf = 0;
volatile int16_t fm_wheel_rpm_lb = 0;
volatile int16_t fm_wheel_rpm_rb = 0;

volatile int16_t fm_wheel_current_lf = 0;
volatile int16_t fm_wheel_current_rf = 0;
volatile int16_t fm_wheel_current_lb = 0;
volatile int16_t fm_wheel_current_rb = 0;

volatile float fm_steer_angle_lf = 0.0f;
volatile float fm_steer_angle_rf = 0.0f;
volatile float fm_steer_angle_lb = 0.0f;
volatile float fm_steer_angle_rb = 0.0f;

volatile float fm_steer_target_lf = 0.0f;
volatile float fm_steer_target_rf = 0.0f;
volatile float fm_steer_target_lb = 0.0f;
volatile float fm_steer_target_rb = 0.0f;

volatile float fm_steer_error_lf = 0.0f;
volatile float fm_steer_error_rf = 0.0f;
volatile float fm_steer_error_lb = 0.0f;
volatile float fm_steer_error_rb = 0.0f;

volatile int32_t fm_steer_current_lf = 0;
volatile int32_t fm_steer_current_rf = 0;
volatile int32_t fm_steer_current_lb = 0;
volatile int32_t fm_steer_current_rb = 0;

volatile float fm_follow_yaw_encoder = 0.0f;
volatile float fm_follow_yaw_error = 0.0f;
volatile float fm_follow_w_raw = 0.0f;
volatile float fm_follow_w = 0.0f;
}

struct wheel_state_angle {
    float lf_angle;
    float rf_angle;
    float lb_angle;
    float rb_angle;
};
struct wheel_target {
    float lf_angle;
    float rf_angle;
    float lb_angle;
    float rb_angle;

    float lf_speed;
    float rf_speed;
    float lb_speed;
    float rb_speed;
};
struct fact_v_angle {
    float lf_angle;
    float rf_angle;
    float lb_angle;
    float rb_angle;

    float lf_speed;
    float rf_speed;
    float lb_speed;
    float rb_speed;
};

// CAN 0x130 uses data[6..7] for this signed 16-bit, big-endian mode.
enum class ChassisMode : int16_t {
    kNoForce = 0,
    kFollow = 1,
    kRotate = 2,
    kReRotate = 3,
};

class chass final : public rm::device::CanDevice {
public:
    explicit chass(rm::hal::CanInterface &can)
        : CanDevice(can, 0x130),
          steer_compute(kChassisRadius),
          yaw_motor_(can, 4),
          follow_pid_(15.0f, 0.0f, 3.7f, 3.0f, 0.0f) {
        follow_pid_.SetCircular(true);
        follow_pid_.SetCircularCycle(2.0f * M_PI);
        yaw_motor_.SetHeartbeatTimeout(std::chrono::milliseconds(100));
    }

    bool update(const wheel_state_angle &wheel) {
        const auto interrupt_mask = __get_PRIMASK();
        __disable_irq();
        const auto command = command_;
        const bool online = received_ && HAL_GetTick() - last_rx_ms_ <= 100U;
        __set_PRIMASK(interrupt_mask);
        fm_chassis_online = static_cast<uint32_t>(online);

        if (!online || yaw_motor_.online_status() != rm::device::Device::kOk) {
            ready_ = false;
        } else if (command.mode == ChassisMode::kNoForce) {
            ready_ = true;
        }
        if (!ready_ || command.mode == ChassisMode::kNoForce) {
            follow_pid_.Clear();
            fm_follow_w_raw = 0.0f;
            fm_follow_w = 0.0f;
            return false;
        }

        float forward = command.y;
        float lateral = command.x;
        const float move_speed = std::hypot(forward, lateral);
        constexpr float kMaxLinearSpeed = 1.40f;
        if (move_speed > kMaxLinearSpeed) {
            const float scale = kMaxLinearSpeed / move_speed;
            forward *= scale;
            lateral *= scale;
        }

        const float yaw_encoder = static_cast<float>(yaw_motor_.encoder());
        const float yaw_error = rm::modules::Wrap(
            kFrontDownYawAngle - yaw_encoder * (2.0f * M_PI / 8192.0f), -M_PI, M_PI);
        float angle = yaw_error;
        float w=0 ;
        if (command.mode == ChassisMode::kFollow) {
            follow_pid_.Update(0.0f, -yaw_error);
            if (std::abs( command_ .w )>0) {
                w=1500*command_.w;
            }
            else {
                w = follow_pid_.out();
            }
        } else {
            follow_pid_.Clear();
            constexpr float kSpinAngleOffset = -0.15f;
            constexpr float kSpinSpeed = 3.5f;
            if (command.mode == ChassisMode::kRotate) {
                angle += kSpinAngleOffset;
                w = kSpinSpeed;
            } else if (command.mode == ChassisMode::kReRotate) {
                angle -= kSpinAngleOffset;
                w = -kSpinSpeed;
            }
        }

        const float cos_yaw = std::cos(angle);
        const float sin_yaw = std::sin(angle);
        const float target_x = forward * cos_yaw - lateral * sin_yaw;
        const float target_y = forward * sin_yaw + lateral * cos_yaw;

        fm_follow_yaw_encoder = yaw_encoder;
        fm_follow_yaw_error = yaw_error;
        fm_follow_w_raw = w;
        fm_follow_w = w;

        const auto result = steer_compute.Forward(
            target_x, target_y, w, wheel.lf_angle, wheel.rf_angle, wheel.lb_angle, wheel.rb_angle);
        constexpr float kWheelRadius = 0.05f;
        constexpr float kGearRatio = 19.0f;
        constexpr float kRpmPerMps = 60.0f * kGearRatio * 0.5f / (2.0f * M_PI * kWheelRadius);
        const float spin = w * kChassisRadius * std::sqrt(0.5f);


        target.lf_speed = kRpmPerMps * std::copysign(
            std::hypot(target_x + spin, target_y + spin), result.lf_wheel_speed);
        target.rf_speed = kRpmPerMps * std::copysign(
            std::hypot(target_x - spin, target_y + spin), result.rf_wheel_speed);
        target.lb_speed = kRpmPerMps * std::copysign(
            std::hypot(target_x + spin, target_y - spin), result.lr_wheel_speed);
        target.rb_speed = kRpmPerMps * std::copysign(
            std::hypot(target_x - spin, target_y - spin), result.rr_wheel_speed);

        target.lf_angle = result.lf_steer_position;
        target.rf_angle = result.rf_steer_position;
        target.lb_angle = result.lr_steer_position;
        target.rb_angle = result.rr_steer_position;

        fm_steer_target_lf = target.lf_angle;
        fm_steer_target_rf = target.rf_angle;
        fm_steer_target_lb = target.lb_angle;
        fm_steer_target_rb = target.rb_angle;
        return true;
    }

    const wheel_target &target_() const { return target; }

    void RxCallback(const rm::hal::CanFrame *frame) override {
        if (frame->rx_std_id != 0x130 || frame->dlc != 8) {
            return;
        }

        const auto raw_mode = GetI16BE(&frame->data[6]);
        ChassisMode mode = ChassisMode::kNoForce;
        if (raw_mode >= static_cast<int16_t>(ChassisMode::kNoForce) &&
            raw_mode <= static_cast<int16_t>(ChassisMode::kReRotate)) {
            mode = static_cast<ChassisMode>(raw_mode);
        }

        command_ = {GetI16BE(&frame->data[0]) / 1000.0f,
                    GetI16BE(&frame->data[2]) / 1000.0f, GetI16BE(&frame->data [4])/1000.0f,mode};
        last_rx_ms_ = HAL_GetTick();
        received_ = true;
        fm_chassis_rx_count = fm_chassis_rx_count + 1U;
    }

private:
    static int16_t GetI16BE(const uint8_t *p) {
        const uint16_t raw = static_cast<uint16_t>(p[0]) << 8 | p[1];
        if (raw & 0x8000U) {
            return static_cast<int16_t>(static_cast<int32_t>(raw) - 65536);
        }
        return static_cast<int16_t>(raw);
    }

    struct Command {
        float x{0.0f};
        float y{0.0f};
        float w{0.0f};
        ChassisMode mode{ChassisMode::kNoForce};
    };

    static constexpr float kChassisRadius = 0.4f;
    static constexpr float kFrontDownYawAngle = 4.1560f;
    rm::modules::SteeringChassis steer_compute;
    rm::device::GM6020 yaw_motor_;
    rm::modules::PID follow_pid_;
    wheel_target target{};
    Command command_{};
    uint32_t last_rx_ms_{0};
    bool received_{false};
    bool ready_{false};
};

///进行编码器数据处理
class Data_deal {
    public:
  Data_deal () = default;
wheel_state_angle  update(const wheel_state_angle &wheel_fact, wheel_state_angle  &wheel_deal) {
    wheel_deal.lb_angle=wheel_fact.lb_angle-lb_zero ;


    // 处理编码器跨越 0/8191 的情况
    if (wheel_deal .lb_angle  > 4096.0f) {
        wheel_deal.lb_angle  -= 8192.0f;
    }
    if (wheel_deal .lb_angle  < -4096.0f) {
        wheel_deal .lb_angle  += 8192.0f;
    }

    // 编码器转换成弧度
    final_deal_wheel .lb_angle  =
        wheel_deal .lb_angle  * (2.0f * M_PI) / 8192.0f;
    ////////////////////////////////////////////////////////
    wheel_deal.rb_angle=wheel_fact.rb_angle-rb_zero ;
    if (wheel_deal .rb_angle  > 4096.0f) {
        wheel_deal.rb_angle  -= 8192.0f;
    }
    if (wheel_deal .rb_angle  < -4096.0f) {
        wheel_deal .rb_angle  += 8192.0f;
    }

    // 编码器转换成弧度

    final_deal_wheel .rb_angle  =
        wheel_deal .rb_angle  * (2.0f * M_PI) / 8192.0f;
    //////////////////////////////////////////////////////////////
    ///
    wheel_deal.lf_angle=wheel_fact.lf_angle-lf_zero ;
    if (wheel_deal .lf_angle  > 4096.0f) {
        wheel_deal.lf_angle  -= 8192.0f;
    }
    if (wheel_deal .lf_angle  < -4096.0f) {
        wheel_deal .lf_angle  += 8192.0f;


        // 编码器转换成弧度
        final_deal_wheel .lf_angle  =
            wheel_deal .lf_angle  * (2.0f * M_PI) / 8192.0f;
    }
    //////////////////////////////////////////////
    final_deal_wheel.lf_angle =
        wheel_deal.lf_angle * (2.0f * M_PI) / 8192.0f;

    wheel_deal.rf_angle=wheel_fact.rf_angle-rf_zero ;
    if (wheel_deal .rf_angle  > 4096.0f) {
        wheel_deal.rf_angle  -= 8192.0f;
    }
    if (wheel_deal .rf_angle  < -4096.0f) {
        wheel_deal .rf_angle  += 8192.0f;
    }

    // 编码器转换成弧度
    final_deal_wheel .rf_angle  =
        wheel_deal .rf_angle  * (2.0f * M_PI) / 8192.0f;

    return final_deal_wheel ;
}

private:
   wheel_state_angle  final_deal_wheel;
    const float lb_zero=2912;//850
    const float rb_zero=6518;
    const float lf_zero=1114;
    const float rf_zero=7129;
};
class Device_motor_PID {
    public:
    Device_motor_PID(

    rm::hal::CanInterface &can,

    uint16_t motor_id1,
    uint16_t motor_id2,
    uint16_t motor_id3,
    uint16_t motor_id4)
        : can_(can),

          motor_lf(can_, motor_id1),
          motor_rf(can_, motor_id2),
          motor_lb(can_, motor_id4),
          motor_rb(can_, motor_id3),
          motor_V_lb(can_, motor_id4),
          motor_V_lf(can_, motor_id1),
          motor_V_rb(can_, motor_id3),
          motor_V_rf(can_, motor_id2),
          steer_lf_pid(1000, 0, 0, 10000, 0),
          steer_rf_pid(1000, 0, 0, 10000, 0),
          steer_lb_pid(1000, 0, 0, 10000, 0),
          steer_rb_pid(1000, 0, 0, 10000, 0),
          steer_lf_I_pid(10, 0, 0, 10000, 0),
          steer_rf_I_pid(10, 0, 0, 10000, 0),
          steer_lb_I_pid(10, 0, 0, 10000, 0),
          steer_rb_I_pid(10, 0, 0, 10000, 0),
          wheel_lf_pid(8, 0, 0, 10000, 0),
          wheel_rf_pid(8, 0, 0, 10000, 0),
          wheel_lb_pid(8, 0, 0, 10000, 0),
          wheel_rb_pid(8, 0, 0, 10000, 0) {
        steer_lf_pid.SetCircular(true).SetCircularCycle(2.0f * M_PI);
        steer_rf_pid.SetCircular(true).SetCircularCycle(2.0f * M_PI);
        steer_lb_pid.SetCircular(true).SetCircularCycle(2.0f * M_PI);
        steer_rb_pid.SetCircular(true).SetCircularCycle(2.0f * M_PI);
    };
////8个pid还有freemaster里面可视变量
    void update(fact_v_angle &data, float dt, wheel_state_angle &angle,
                wheel_target all, float power_scale)
    {
        steer_lf_pid.Update(all.lf_angle, angle.lf_angle, dt);
        steer_rf_pid.Update(all.rf_angle, angle.rf_angle, dt);
        steer_lb_pid.Update(all.lb_angle, angle.lb_angle, dt);
        steer_rb_pid.Update(all.rb_angle, angle.rb_angle, dt);

        steer_lf_I_pid.Update(
            steer_lf_pid.out(),
            static_cast<float>(motor_lf.rpm()),
            dt);
        steer_rf_I_pid.Update(
            steer_rf_pid.out(),
            static_cast<float>(motor_rf.rpm()),
            dt);
        steer_lb_I_pid.Update(
            steer_lb_pid.out(),
            static_cast<float>(motor_lb.rpm()),
            dt);
        steer_rb_I_pid.Update(
            steer_rb_pid.out(),
            static_cast<float>(motor_rb.rpm()),
            dt);

        fm_steer_error_lf = all.lf_angle - angle.lf_angle;
        fm_steer_error_rf = all.rf_angle - angle.rf_angle;
        fm_steer_error_lb = all.lb_angle - angle.lb_angle;
        fm_steer_error_rb = all.rb_angle - angle.rb_angle;

        wheel_lf_pid.Update(all.lf_speed, data.lf_speed, dt);
        wheel_rf_pid.Update(all.rf_speed, data.rf_speed, dt);
        wheel_lb_pid.Update(all.lb_speed, data.lb_speed, dt);
        wheel_rb_pid.Update(all.rb_speed, data.rb_speed, dt);

        fm_steer_current_lf = static_cast<int32_t>(steer_lf_I_pid.out() * power_scale);
        fm_steer_current_rf = static_cast<int32_t>(steer_rf_I_pid.out() * power_scale);
        fm_steer_current_lb = static_cast<int32_t>(steer_lb_I_pid.out() * power_scale);
        fm_steer_current_rb = static_cast<int32_t>(steer_rb_I_pid.out() * power_scale);

        motor_lf.SetCurrent(static_cast<int16_t>(fm_steer_current_lf));
        motor_rf.SetCurrent(static_cast<int16_t>(fm_steer_current_rf));
        motor_lb.SetCurrent(static_cast<int16_t>(fm_steer_current_lb));
        motor_rb.SetCurrent(static_cast<int16_t>(fm_steer_current_rb));

        motor_V_lf.SetCurrent(static_cast<int16_t>(wheel_lf_pid.out() * power_scale));
        motor_V_rf.SetCurrent(static_cast<int16_t>(wheel_rf_pid.out() * power_scale));
        motor_V_lb.SetCurrent(static_cast<int16_t>(wheel_lb_pid.out() * power_scale));
        motor_V_rb.SetCurrent(static_cast<int16_t>(wheel_rb_pid.out() * power_scale));

    }

    wheel_state_angle  get_angle() const {
        const wheel_state_angle  raw = {
            static_cast<float>(motor_lf.encoder()),
            static_cast<float>(motor_rf.encoder()),
            static_cast<float>(motor_lb.encoder()),
            static_cast<float>(motor_rb.encoder())
        };

        fm_steer_raw_lf = raw.lf_angle;
        fm_steer_raw_rf = raw.rf_angle;
        fm_steer_raw_lb = raw.lb_angle;
        fm_steer_raw_rb = raw.rb_angle;

        return raw;
    }

    fact_v_angle  get_speed() const {
        fm_wheel_encoder_lf = motor_V_lf.encoder();
        fm_wheel_encoder_rf = motor_V_rf.encoder();
        fm_wheel_encoder_lb = motor_V_lb.encoder();
        fm_wheel_encoder_rb = motor_V_rb.encoder();

        fm_wheel_rpm_lf = motor_V_lf.rpm();
        fm_wheel_rpm_rf = motor_V_rf.rpm();
        fm_wheel_rpm_lb = motor_V_lb.rpm();
        fm_wheel_rpm_rb = motor_V_rb.rpm();

        fm_wheel_current_lf = motor_V_lf.current();
        fm_wheel_current_rf = motor_V_rf.current();
        fm_wheel_current_lb = motor_V_lb.current();
        fm_wheel_current_rb = motor_V_rb.current();

        return {
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            static_cast<float>(motor_V_lf.rpm()),
            static_cast<float>(motor_V_rf.rpm()),
            static_cast<float>(motor_V_lb.rpm()),
            static_cast<float>(motor_V_rb.rpm())
        };
    }

    void send() {
        rm::device::DjiMotorBase::SendCommand(can_);
    }

    void stop() {
        motor_lf.SetCurrent(0);
        motor_rf.SetCurrent(0);
        motor_lb.SetCurrent(0);
        motor_rb.SetCurrent(0);

        motor_V_lf.SetCurrent(0);
        motor_V_rf.SetCurrent(0);
        motor_V_lb.SetCurrent(0);
        motor_V_rb.SetCurrent(0);

        send();
    }

private:
    rm::hal::CanInterface &can_;

    rm::device::GM6020 motor_lf;
    rm::device::GM6020 motor_rf;
    rm::device::GM6020 motor_lb;
    rm::device::GM6020 motor_rb;
    rm::device::M3508 motor_V_lb;
    rm::device::M3508 motor_V_lf;
    rm::device::M3508 motor_V_rb;
    rm::device::M3508 motor_V_rf;

    rm::modules::PID steer_lf_pid;
    rm::modules::PID steer_rf_pid;
    rm::modules::PID steer_lb_pid;
    rm::modules::PID steer_rb_pid;

    rm::modules::PID steer_lf_I_pid;
    rm::modules::PID steer_rf_I_pid;
    rm::modules::PID steer_lb_I_pid;
    rm::modules::PID steer_rb_I_pid;

    rm::modules::PID wheel_lf_pid;
    rm::modules::PID wheel_rf_pid;
    rm::modules::PID wheel_lb_pid;
    rm::modules::PID wheel_rb_pid;

};
class App {
public:
    explicit App(rm::hal::CanInterface &can,rm::hal:: CanInterface &can_)
        : chassis_(can),
          angle_data(),
          device_(can_, 1, 2, 3, 4),
          super_cap_(can) {
    }

    void Init() {


    }

    void update(float dt) {
        const float power_scale = PowerLimitLoop();
        const wheel_state_angle  raw_angle = device_.get_angle();

        wheel_state_angle  calibrated_encoder{};
        wheel_state_angle  current_angle =
            angle_data .update(raw_angle, calibrated_encoder);//获得处理过的当前角度值

        fm_steer_angle_lf = current_angle.lf_angle;
        fm_steer_angle_rf = current_angle.rf_angle;
        fm_steer_angle_lb = current_angle.lb_angle;
        fm_steer_angle_rb = current_angle.rb_angle;

        if (!chassis_.update(current_angle)) {
            device_.stop();
            return;
        }

        fact_v_angle  wheel_speed = device_.get_speed();
        device_.update(wheel_speed, dt,
                       current_angle,
                       chassis_.target_(), power_scale);

        device_.send();
    }

private:
    float PowerLimitLoop() {
        RefereeChassisPower power{};
        rm::device::GkSupercap::TxData super_cap_tx{};
        float power_scale = 1.0f;

        if (ReadRefereeChassisPower(power)) {
            referee_power_seen_ = true;
            super_cap_tx.enable_dcdc = true;
            super_cap_tx.feedback_referee_energy_buffer = power.buffer_energy;
            if (power.buffer_energy < 20) {
                power_scale = 0.0f;
            } else if (power.buffer_energy < 50) {
                const float scale = static_cast<float>(power.buffer_energy) / 60.0f;
                power_scale = scale * scale;
                super_cap_tx.feedback_referee_power_limit = power.power_limit / 2;
            } else {
                super_cap_tx.feedback_referee_power_limit = power.power_limit;
            }
        } else if (referee_power_seen_) {

            power_scale = 0.0f;
        }

        fm_chassis_power_scale = power_scale;
        const auto now = HAL_GetTick();

        if (referee_power_seen_ && now - last_super_cap_tx_ms_ >= kSuperCapTxPeriodMs &&
            HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
            super_cap_.Update(super_cap_tx);
            last_super_cap_tx_ms_ = now;
            fm_super_cap_tx_power_limit = super_cap_tx.feedback_referee_power_limit;
            fm_super_cap_tx_count = fm_super_cap_tx_count + 1;
        }
        return power_scale;
    }

    static constexpr uint32_t kSuperCapTxPeriodMs = 2;
    chass chassis_;
    Data_deal  angle_data;
    Device_motor_PID device_;
    rm::device::GkSupercap super_cap_;
    uint32_t last_super_cap_tx_ms_{0};
    bool referee_power_seen_{false};
};

static App *g_app = nullptr;

void ChassisAppInit(rm::hal::CanInterface &can,rm::hal::CanInterface &can_) {
    g_app = new App(can,can_);
    g_app->Init();
}

void ChassisAppUpdate(float dt) {
    if (g_app != nullptr) {
        g_app->update(dt);
    }
}
