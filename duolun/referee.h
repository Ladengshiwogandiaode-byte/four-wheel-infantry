//
// Created by ningj on 2026/11/19.
//

#ifndef DUOLUN_REFEREE_H
#define DUOLUN_REFEREE_H

#include <cstdint>

struct RefereeChassisPower {
    std::uint16_t power_limit;
    std::uint16_t buffer_energy;
};

void RefereeInit();
void RefereeUpdate();

// 两种功率数据都有效时才更新 power。
bool ReadRefereeChassisPower(RefereeChassisPower &power);

#endif //DUOLUN_REFEREE_H
