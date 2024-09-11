#pragma once 
#include <string>

#include "robot/intake.h"
#include "robot/clamp.h"

namespace Robot {

    class Autonomous{
        public:
            enum routine {redLeft = 1, redRight = 2, skills = 0, blueLeft = -1, blueRight = -2};
            
            static routine auton;

            static std::string autonName;

            void autonMove(Intake &intake, Clamp &clamp);

            static void autonSwitcher(int routine);
    };

}