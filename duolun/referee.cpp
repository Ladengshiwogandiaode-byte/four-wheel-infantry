//
// Created by ningj on 2026/11/19.
//
#include "referee.h"

#include "librm/device/referee/referee.hpp"
#include "usart.h"

extern "C" {
volatile std::uint32_t fm_referee_rx_count = 0;
volatile std::uint8_t fm_referee_robot_id = 0;
volatile std::uint16_t fm_referee_current_hp = 0;
volatile std::uint16_t fm_referee_max_hp = 0;
volatile std::uint16_t fm_referee_chassis_power_limit = 0;
volatile std::uint16_t fm_referee_buffer_energy = 0;
volatile std::uint8_t fm_referee_chassis_output = 0;
volatile std::uint8_t fm_referee_power_valid = 0;
}

namespace {
constexpr auto kRevision = rm::device::RefereeRevision::kNewV200;
constexpr std::uint32_t kPowerTimeoutMs = 500;
using CmdId = rm::device::RefereeCmdId<kRevision>;

rm::device::Referee<kRevision> referee;
std::uint8_t rx_buffer[2][rm::device::kRefProtocolFrameMaxLen];
std::uint8_t rx_index = 0;
///收到数据并且不超时
struct FrameTime {
    std::uint32_t tick = 0;
    bool received = false;

    bool IsFresh(std::uint32_t now) const {
        return received && now - tick <= kPowerTimeoutMs;
    }
};

FrameTime robot_status_time;
FrameTime power_heat_time;
///开始接收
void StartReceive() {
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buffer[rx_index], sizeof(rx_buffer[0])) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
    }
}

void OnRefereeFrame(std::uint16_t cmd_id, std::uint8_t) {
    fm_referee_rx_count = fm_referee_rx_count + 1;
    if (cmd_id == CmdId::kRobotStatus) {
        const auto &status = referee.data().robot_status;
        fm_referee_robot_id = status.robot_id;
        fm_referee_current_hp = status.current_HP;
        fm_referee_max_hp = status.maximum_HP;
        fm_referee_chassis_power_limit = status.chassis_power_limit;
        fm_referee_chassis_output = status.power_management_chassis_output;
        robot_status_time = {HAL_GetTick(), true};
    } else if (cmd_id == CmdId::kPowerHeatData) {
        fm_referee_buffer_energy = referee.data().power_heat_data.buffer_energy;
        power_heat_time = {HAL_GetTick(), true};
    }
}
}  // namespace


    void RefereeInit() {
        referee.AttachCallback(OnRefereeFrame);
        StartReceive();
    }




void RefereeUpdate() {


    RefereeChassisPower power{};
    fm_referee_power_valid = ReadRefereeChassisPower(power);
}

bool  ReadRefereeChassisPower(RefereeChassisPower &power) {
    // 状态帧和功率帧由中断更新，在同一个临界区检查时间并复制。
    const auto interrupts = __get_PRIMASK();
    __disable_irq();
    const auto now = HAL_GetTick();
    const bool fresh = robot_status_time.IsFresh(now) && power_heat_time.IsFresh(now);
    if (fresh) {
        const auto &data = referee.data();
        power = {data.robot_status.chassis_power_limit, data.power_heat_data.buffer_energy};
    }
    __set_PRIMASK(interrupts);
    return fresh;
}
////收到信息发给librm处理
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, std::uint16_t size) {
    if (huart != &huart6 || HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) {
        return;
    }

    // 用另一块缓冲区接收后续字节，同时解析刚收到的字节。
    const auto &received = rx_buffer[rx_index];
    rx_index ^= 1u;
    StartReceive();

    for (std::uint16_t i = 0; i < size; ++i) {
        referee << received[i];
    }
}
