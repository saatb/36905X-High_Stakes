#pragma once 
#include <string>

#include "robot/intake.h"
#include "robot/clamp.h"

namespace Robot {

    class Autonomous{
        public:
            enum routine { redLeft = 1, redRight = 2, blueLeft = -1, blueRight = -2, skills = 0, };
            
            static routine auton;

            static std::string autonName;

            void autonMove(Intake &intake, Clamp &clamp);

            static void autonSwitcher(int autonNum);

        private:
            void auton1(Intake &intake, Clamp &clamp);

            void auton2(Intake &intake, Clamp &clamp);
            
            void auton3(Intake &intake, Clamp &clamp);
            
            void auton4(Intake &intake, Clamp &clamp);
            
            void auton5(Intake &intake, Clamp &clamp);
    };

}