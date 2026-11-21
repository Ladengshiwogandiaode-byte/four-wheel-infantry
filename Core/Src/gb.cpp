//
// Created by ningj on 2026/10/30.
//

#include "gb.h"
#include <librm.hpp>
#include "can.h"
#include "usart.h"
#include "math.h"
#include "spi.h"
#include "gb_Pro.cpp"

extern "C" {
volatile float fm_gimbal_yaw_current = 0.0f;
volatile float fm_gimbal_yaw_target = 0.0f;
volatile uint16_t fm_gimbal_yaw_encoder = 0;
volatile int16_t fm_gimbal_yaw_rpm = 0;
volatile int32_t fm_gimbal_yaw_output = 0;
volatile float fm_gimbal_yaw_current_deg = 0.0f;
volatile float fm_gimbal_yaw_target_deg = 0.0f;
volatile float fm_gimbal_imu_yaw_deg = 0.0f;

volatile uint16_t fm_fire_left_encoder = 0;
volatile uint16_t fm_fire_right_encoder = 0;
volatile uint16_t fm_fire_bopan_encoder = 0;
volatile uint16_t fm_gimbal_yaw_po_out = 0;
volatile int32_t fm_gb_left = 0;
volatile int32_t fm_gb_right = 0;
}

static rm::hal::Can *can1=nullptr ;
static rm::hal::Can *can2=nullptr ;
static rm::device:: BMI088 *imu=nullptr ;
static rm::hal::Serial< 36> *dbus=nullptr ;
static rm::device:: DR16 *Dt7=nullptr ;
static rm::modules:: MahonyAhrs *ahrs=nullptr ;
static App *app=nullptr;
extern "C" void Init() {
    can1=new rm::hal::Can (hcan1);
    can2=new rm::hal::Can (hcan2);
    imu=new rm::device:: BMI088 (   hspi1,
        GPIOA, GPIO_PIN_4,
        GPIOB, GPIO_PIN_0);
    dbus=new rm::hal::Serial< 36>(huart3 ,false ,true);
    Dt7=new rm::device:: DR16 (*dbus);
    ahrs =new rm::modules:: MahonyAhrs (500);
    app=new App(*can1,*can2,*Dt7,*imu,*ahrs);

    can1->SetFilter(0, 0);
    can2->SetFilter(0, 0);
    can1->Begin();
    can2->Begin();
    Dt7->Begin();

    HAL_Delay(100);
    app->Init() ;
}
extern "C" void Main() {
    app->update(0.002);
}
