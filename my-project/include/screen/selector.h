#pragma once
#include "pros/apix.h"
#include "robot/auton.h"

namespace Robot{
    
    class autonSelectorScreen{
    public:
        
        autonSelectorScreen();

        void selector();

    private:
    static Autonomous::routine lastAuton;

    constexpr static char redAutons[] = "Red Left\nRed Right\nRed Left AWP\nRed Elims";

    constexpr static char blueAutons[] = "Blue Left\nBlue Right\nBlue Right AWP\nBlue Elims";

    static void autonUiUpdate(lv_event_t *e);
    };
};