//
// Created by ningj on 2026/11/7
#include <librm.hpp>

#include <chrono>
#include <cmath>
#include <utility>
//freemaster可视变量
extern "C" {
extern volatile float fm_gimbal_yaw_current;
extern volatile float fm_gimbal_yaw_target;
extern volatile uint16_t fm_gimbal_yaw_encoder;
extern volatile int16_t fm_gimbal_yaw_rpm;
extern volatile int32_t fm_gimbal_yaw_output;
extern volatile float fm_gimbal_yaw_current_deg;
extern volatile float fm_gimbal_yaw_target_deg;
extern volatile float fm_gimbal_imu_yaw_deg;

extern volatile uint16_t fm_fire_left_encoder;
extern volatile uint16_t fm_fire_right_encoder;
extern volatile uint16_t fm_fire_bopan_encoder;
 extern  volatile  uint16_t fm_gimbal_yaw_po_out;
}

struct Input{
    float Vx_left{0};
    float Vy_left{0};
    float Vx_right{0};
    float Vy_right{0};
    int Key_left{0};
    int Key_right{0};
    float dial{0};
    bool fire_single{false};   // 鼠标左键：单发
    bool fire_auto{false};     // 鼠标右键：连发
};
struct attitude {
    float yaw{0};
    float roll{0};
    float pitch{0};
};


enum class ChassisMode : int16_t {
    kNoForce = 0,
    kFollow = 1,
    kRotate = 2,
    kReRotate = 3,
};

/////////////////////////////////////////////////////////////////////////////

class Dt7_control {
    public:

    Dt7_control(rm::device::DR16 &rc) : rc_(rc) {
        rc_.SetHeartbeatTimeout(std::chrono::milliseconds(100));
    }

    bool Online() { return rc_.online_status() == rm::device::Device::kOk; }
    Input Read (){
        Input input;
        input.Vx_right =normal(rc_.right_x()) ;
        input.Vx_left =-normal(rc_.left_x() );
        input.Vy_right =normal(rc_.right_y()) ;
        input.Vy_left =-normal(rc_.left_y()) ;
        input.Key_left =static_cast<int>(rc_.switch_l()) ;
        input.Key_right=static_cast<int>(rc_.switch_r()) ;

        input.dial =(static_cast<float>(rc_.dial() ))/660.0*2 ;
        input.fire_single = rc_.mouse_button_left() ;
        input.fire_auto = rc_.mouse_button_right() ;

return input;
    };
private:
    rm::device::DR16 &rc_;
    static float normal(float t) {
        if (t<20 && t>-20) {
            return 0;
        }
        return t / 660.0f ;
    }
};

/////////////////////////////////////////////////////////////////////
class attitude_control {
public:

    attitude_control(
           rm::device::BMI088 &imu,
           rm::modules::MahonyAhrs &mahony): imu_(imu), mahony_(mahony) {};

    void update(){
        imu_.Update() ;
        mahony_.Update(rm::modules::ImuData6Dof{
            imu_.gyro_y(),
             imu_.gyro_z(),
             imu_.gyro_x() + 0.0028f,
            imu_.accel_y(),
             imu_.accel_z(),
             imu_.accel_x()
        });
data_.pitch =mahony_.euler_angle().pitch ;
        data_.roll =mahony_.euler_angle().roll ;
        data_.yaw =mahony_.euler_angle().yaw ;




    };
    const attitude attitude_get() const{
        return data_ ;
    }
private:
    rm::device::BMI088 &imu_;
    rm::modules::MahonyAhrs &mahony_;
    attitude data_;
};
////////////////////////////////////////////////////////////////
class Yaw_pid {
    public:
    Yaw_pid (



    rm::hal::CanInterface &can,uint16_t motor_id):can_(can),motor_(can,motor_id),
   position_PID(3600, 0, 38000, 500, 0),
   V_PID(120, 0, 0, 28000, 0) {
        position_PID.SetCircular(true);
        position_PID .SetCircularCycle(2*M_PI );
        V_PID.SetDiffLpfAlpha(0.1f);
    };
    void update(const Input &input,float dt,const attitude &attitude_data,const Input &inout) {
        current_angle_ = attitude_data.yaw;
        current_rpm_ = motor_.rpm();
        current_encoder = motor_.encoder();

        fm_gimbal_yaw_current = current_angle_;
        fm_gimbal_yaw_encoder = motor_.encoder();
        fm_gimbal_yaw_rpm = current_rpm_;
        fm_gimbal_yaw_current_deg =
            current_angle_ * (180.0f / M_PI);


        if (!first_update) {
            target_angle_ = current_angle_;
          first_update  = true;
        }

        if (input.Vx_left != 0) {
            target_angle_ += input.Vx_left * 0.006;

        }


        target_angle_ = rm::modules::Wrap(
            target_angle_,
            0.0f,
            2.0f * M_PI);

        Tar_Yaw = target_angle_;
        curr_yaw = current_angle_;

        position_PID .Update(Tar_Yaw ,curr_yaw );
        V_PID.Update(position_PID .out()  ,current_rpm_ );
        fm_gimbal_yaw_target = target_angle_;
        fm_gimbal_yaw_target_deg =
            target_angle_ * (180.0f / M_PI);
        fm_gimbal_yaw_output =
            static_cast<int32_t>(V_PID.out());
        fm_gimbal_yaw_po_out=
            static_cast <int32_t >(position_PID .out() );
        if ((inout .Key_left !=2)) {
            motor_.SetCurrent( static_cast <int16_t>(V_PID.out() ));
        }
        else  if ((inout .Key_left==2)&&inout .Key_right ==1) {
            motor_.SetCurrent( static_cast <int16_t>(V_PID.out() ));

        }
        else{ motor_.SetCurrent( 0);}


    }
void Send() {
        rm::device::DjiMotorBase ::SendCommand( can_);
    }
 float  Get_encoder()const{
return current_encoder;
    }
private:
    rm::hal::CanInterface &can_;
    rm::device::GM6020 motor_;
    rm::modules::PID position_PID;
    rm::modules::PID V_PID;
    float target_angle_{};
    float current_angle_{};
    int16_t current_rpm_{};
    float current_encoder{};
    float Tar_Yaw{0};
    float curr_yaw{};
    bool first_update{false};
    bool yaw_target_locked_{false};
    uint16_t yaw_stable_ticks_{0};
};


class cs_pid {
public:
    explicit cs_pid(Yaw_pid &yaw) : yaw_(yaw) ,w_pid( 0.03f,0,0.01,2,0) {
        w_pid.SetCircular(true );
        w_pid .SetCircularCycle(8192.0 );
        w_pid.SetDiffFirst(true);
        w_pid.SetDiffLpfAlpha(0.2f);
    };

    float cs_PID(float dt){
        current_encoder =yaw_.Get_encoder() ;

        const float error = rm::modules::Wrap(
            target_encoder - current_encoder,
            -4096.0f,
            4096.0f);

        if (error< 15&&error>=-15) {
            last_output_ *= 0.9f;
            if (-0.01<=last_output_ &&last_output_ <= 0.01f) {
                last_output_ = 0.0f;
            }
            return last_output_;
        }

        w_pid.Update(target_encoder ,current_encoder ,dt );

        constexpr float kMaxWStep = 0.04f;
        const float output = rm::modules::Clamp(
            w_pid.out(),
            last_output_ - kMaxWStep,
            last_output_ + kMaxWStep);

        last_output_ = output;
        return output;
    }
private:
    float target_encoder{3200};
    float current_encoder{};
    float last_output_{0.0f};
    rm::modules:: PID w_pid;
    Yaw_pid &yaw_;
};

//////////////////////////////////////////////////////////////////
class Pitch_pid {
public:
    explicit Pitch_pid (rm::hal::CanInterface &can):motor_ (can, {
              0x03, 0x02,
              12.5f, 30.0f, 10.0f,
              {0.0f, 500.0f},
              {0.0f, 5.0f}}){};

void update(const Input &input,    const attitude &attitude_data,const Input &inout) {
    current_angle_ = motor_.pos();

    const uint32_t now = HAL_GetTick();
    const uint8_t status = motor_.status();

    if (status != 1U) {
        if ((now - last_status_check_) >= 100U) {
            last_status_check_ = now;

            if (status == 0U) {
                motor_.SendInstruction(
                    rm::device::DmMotorInstructions::kEnable);
            } else {
                motor_.SendInstruction(
                    rm::device::DmMotorInstructions::kClearError);
            }
        }
        return;
    }

    if (input.Vy_left !=0) {
        target_angle_ =current_angle_ + input.Vy_left *0.12f;



    };


    fm_gimbal_imu_yaw_deg =
           float( attitude_data.yaw) * (180.0f / M_PI);
    target_angle_ = rm::modules::Clamp(
        target_angle_, kPitchMin, kPitchMax);
    if (!filter_Ready ) {
        filter_ =attitude_data .pitch ;
        filter_Ready =true;
    }
    constexpr float kPitchFilterAlpha = 0.05;
   filter_ +=
    kPitchFilterAlpha *
    (attitude_data.pitch - filter_);
    const float gravity_ff =
             0.9 * cosf(filter_ );
    if ((inout .Key_left !=2)) {
        motor_.SetMitCommand(  target_angle_,
               0.0f,
               -gravity_ff,
               15.0f,
               1.0f);
    }
    else  if ((inout .Key_left==2)&&inout .Key_right ==1) {
        motor_.SetMitCommand(  target_angle_,
             0.0f,
             -gravity_ff,
             28.0f,
             1.0f);

    }
    else{  motor_.SetMitCommand(  0,
            0.0f,
            -gravity_ff,
            22.0f,
            1.0f);}



};


    void Init() {
    target_angle_ =motor_.pos() ;
    motor_.SendInstruction(rm::device::DmMotorInstructions::kEnable);
    }

private:
    rm::device:: DmMotorMit motor_;
float target_angle_{};
    float current_angle_{};
    uint32_t last_status_check_{0};
    float filter_{0};
bool filter_Ready{false};
    static constexpr float kPitchMin = -0.7f;
    static constexpr float kPitchMax = 0.2f;
};
/////////////////////////////////////////////////////////////
// CAN 0x130 uses data[6..7] for this signed 16-bit, big-endian mode.


class Chass {
public:
    Chass(rm::hal::CanInterface &can, Dt7_control &remote) : can_(can), remote_(remote) {}

    void Send(const Input &input) {
        using Switch = rm::device::DR16::SwitchPosition;
        const auto left = static_cast<Switch>(input.Key_left);
        const auto right = static_cast<Switch>(input.Key_right);

w=input.dial ;
        ChassisMode mode = ChassisMode::kNoForce;
        if (!remote_.Online() ) {
            ready_ = false ;
        }
        else {
            ready_ = true ;
        }
        if (ready_ ) {
            switch (left) {
                case Switch::kMid:
                    if (right  == Switch::kMid) {
                        mode = ChassisMode::kFollow ;
                    }
                    else if (right == Switch::kUp ) {
                        mode = ChassisMode::kRotate  ;

                    }
                    else {
                        mode = ChassisMode::kReRotate  ;
                    }
                    break;
                case Switch::kUp:

                    mode = ChassisMode::kFollow;
                    break;
                case Switch::kDown:

                    mode = ChassisMode::kNoForce ;
                    break;
                case Switch::kUnknown:
                    break;
            }
        }


        uint8_t data[8]{};
        if (mode != ChassisMode::kNoForce) {
            constexpr float kMaxLinearSpeedMmPerSecond = 1600.0f;
            const auto x = static_cast<int16_t>(std::lround(
                rm::modules::Clamp(input.Vx_right, -1.0f, 1.0f) * kMaxLinearSpeedMmPerSecond));
            const auto y = static_cast<int16_t>(std::lround(
                rm::modules::Clamp(input.Vy_right, -1.0f, 1.0f) * kMaxLinearSpeedMmPerSecond));
            PutI16BE(&data[0], x);
            PutI16BE(&data[2], y);
        }
PutI16BE(&data[4], static_cast<int16_t>(w));
        PutI16BE(&data[6], static_cast<int16_t>(mode));
        can_.Write(0x130, data, sizeof(data));
        last_mode_ = mode;

        sent_ = true;
    }

private:
    static void PutI16BE(uint8_t *data, int16_t value) {
        const auto raw = static_cast<uint16_t>(value);
        data[0] = static_cast<uint8_t>(raw >> 8);
        data[1] = static_cast<uint8_t>(raw);
    }

    rm::hal::CanInterface &can_;
    Dt7_control &remote_;
    bool ready_{false};
    bool sent_{false};
    ChassisMode last_mode_{ChassisMode::kNoForce};
    uint32_t last_tx_ms_{0};
    float w{0};
};
class fire {
public:
  
    fire(rm::hal::CanInterface &can, rm::hal::CanInterface &can2)
        : can_(can),
          can2_(can2),
          Left_V_PID(16.0f, 0.0f, 0.0f, 16384.0f, 0.0f),
          Right_V_PID(16.0f, 0.0f, 0.0f, 16384.0f, 0.0f),
          Bopan_position_PID(500.0f, 0.0f, 10.0f, 2000.0f, 0.0f),
          Bopan_V_PID(8.0f, 0.0f, 0.0f, 10000.0f, 0.0f),
          Left_(can_, 1),
          Right_(can_, 2),
          Bopan(can2_, 5) {}

    void update(const Input &input, float dt) {

        bopan_counter_.Update(static_cast<float>(Bopan.encoder()));
        const float fact_angle =
            bopan_counter_.real_position() / 8192.0f * 2.0f * M_PI;

        const bool single = ((input.Key_left ==1 && input.Key_right ==1)||(input .Key_left ==2&&input .Key_right ==1
            ) )&&(input.dial >= kjudge) ;
        const bool repeat =((input.Key_left ==1 && input.Key_right ==1)||(input .Key_left ==2&&input .Key_right ==1
            ) )&&(input.dial <=- kjudge) ;


        const bool firing_cmd = single || repeat;
        const uint32_t now = HAL_GetTick();
        if (firing_cmd && !fric_on_) {
            fric_on_ms_ = now;
        }//只记录一次时间
        fric_on_ = firing_cmd;
        const bool ready_to_feed = firing_cmd && (now - fric_on_ms_ >= Wait );///等待时间


        if (firing_cmd) {
            Left_V_PID.Update(-kRpm , Left_.rpm(), dt);
            Right_V_PID.Update(kRpm, Right_.rpm(), dt);
            Out_l = Left_V_PID.out();
            Out_r = Right_V_PID.out();
        } else {
            Out_l = 0.0f;
            Out_r = 0.0f;
        }

        if (repeat) {
            Flag_fire_over = 0;
            // 连发：等摩擦轮起速后再持续送弹

            Bopan_V_PID.Update(ready_to_feed ?  k_in_a_row_rpm  : 0.0f, Bopan.rpm(), dt);
            Out_Bopan = Bopan_V_PID.out();
        } else if (single) {
            if (!ready_to_feed) {
                // 摩擦轮还没到速，拨盘先别动
                Bopan_V_PID.Update(0.0f, Bopan.rpm(), dt);
            } else {
                // 单发：从起速那一刻的位置开始，转够一发就停
                if (Flag_fire_over == 0) {
                    Flag_fire_over = 1;
                    target_bopan_ = fact_angle + kDialDirection * kone_angle ;
                }
                Bopan_position_PID.Update(target_bopan_, fact_angle, dt);
                Bopan_V_PID.Update(Bopan_position_PID.out(), Bopan.rpm(), dt);
            }
            Out_Bopan = Bopan_V_PID.out();
        } else {
            Flag_fire_over = 0;
            Out_Bopan = 0.0f;
        }

        // 给 FreeMaster 读的三个电机编码器值
        fm_fire_left_encoder = Left_.encoder();
        fm_fire_right_encoder = Right_.encoder();
        fm_fire_bopan_encoder = Bopan.encoder();

        Left_.SetCurrent(static_cast<int16_t>(Out_l));
        Right_.SetCurrent(static_cast<int16_t>(Out_r));
        Bopan.SetCurrent(static_cast<int16_t>(Out_Bopan));
    }


    void Send() { rm::device::DjiMotorBase::SendCommand(); }

private:
    rm::hal::CanInterface &can_;
    rm::hal::CanInterface &can2_;

    rm::modules::PID Left_V_PID;
    rm::modules::PID Right_V_PID;
    rm::modules::PID Bopan_position_PID;
    rm::modules::PID Bopan_V_PID;

    rm::device::M3508 Left_;
    rm::device::M3508 Right_;
    rm::device::M3508 Bopan;

    rm::modules::EncoderCounter bopan_counter_{8192.0f};  // 拨盘多圈位置

    int Flag_fire_over{0};
    float target_bopan_{0.0f};
    float Out_l{0};
    float Out_r{0};
    float Out_Bopan{0};

    bool fric_on_{false};
    uint32_t fric_on_ms_{0};


    static constexpr uint32_t Wait= 200;
    static constexpr float kRpm = 3200.0f;
    static constexpr float k_in_a_row_rpm= 1500.0f;      // 连发时拨盘转速(rpm)
    static constexpr float kDialDirection = 1.0f;
    static constexpr float kone_angle = 11.6f;
    static constexpr float kjudge = 0.5f;
};
class gimbal {
    public:
    gimbal(
      rm::hal::CanInterface &can1,
      rm::hal::CanInterface &can2)
      : yaw_(can1, 4),
         pitch_(can2),cs_(yaw_),
    fire_(can2,can1) {}
    void Init() {
        pitch_.Init() ;
    }

    void update(const Input &input,const attitude &attitude_data, float dt) {
        yaw_.update(input,dt,attitude_data,input   );
        pitch_.update(input,attitude_data,input   );
        fire_.update(input,dt   );

    }
    void send(){
        yaw_.Send();
        fire_.Send();
    }
    cs_pid &follow_pid() {
        return cs_;
    }
    private:
     Yaw_pid yaw_;
    Pitch_pid pitch_;
cs_pid cs_;
fire fire_;
};


class App {
    public:
    App(rm::hal::CanInterface &can1,
        rm::hal::CanInterface &can2,
        rm::device::DR16 &rc,
        rm::device::BMI088 &imu,
        rm::modules::MahonyAhrs &ahrs)
        : attitude_(imu, ahrs),
          remote_(rc),
          gimbal_(can1, can2),
          chass_(can1, remote_) {};
    void Init() {
        gimbal_.Init();
    };
    void update(float dt) {
        const Input input=remote_.Read() ;
        attitude_.update() ;
       att_= attitude_.attitude_get();
        gimbal_.update(input,att_,dt);
        gimbal_ .send();
        chass_.Send(input);
    }


      private:
    attitude_control attitude_;
    Dt7_control remote_;
    gimbal gimbal_;
    attitude att_;
    Chass chass_;

};

