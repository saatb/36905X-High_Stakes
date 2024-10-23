#pragma once
#include "pros/apix.h"
#include "robot/auton.h"

namespace Robot{
    class autonSelectorScreen{
    public:
        
        void selector();

    private:
    static Autonomous::routine lastAuton;

    constexpr static char redAutons[] = "Red Left\nRed Right";

    constexpr static char blueAutons[] = "Blue Left\nBlue Right";

    static void autonUiUpdate(lv_event_t *e);
    };
};