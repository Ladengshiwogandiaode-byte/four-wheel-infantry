#ifndef CHSS_PRO_HPP
#define CHSS_PRO_HPP

#include <librm.hpp>

void ChassisAppInit(rm::hal::CanInterface &comm_can,
                    rm::hal::CanInterface &motor_can);
void ChassisAppUpdate(float dt);

#endif  // CHSS_PRO_HPP
