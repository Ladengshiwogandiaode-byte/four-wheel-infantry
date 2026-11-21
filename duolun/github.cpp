#include <librm.hpp>

#include "can.h"
#include "chss_pro.hpp"
#include "referee.h"

namespace {
rm::hal::Can *can2 = nullptr;
    rm::hal::Can *can1 = nullptr;
}

extern "C" void UserInit(void) {
    can2 = new rm::hal::Can(hcan2);
can1=new rm::hal::Can(hcan1);

    ChassisAppInit(*can1,*can2);

    can1->SetFilter(0, 0);
    can1->Begin();

    can2->SetFilter(0, 0);
    can2->Begin();

    RefereeInit();
}

extern "C" void UserMain(void) {
    RefereeUpdate();
    ChassisAppUpdate(0.001f);
}

// Previous standalone implementation: github_old.cpp.txt
