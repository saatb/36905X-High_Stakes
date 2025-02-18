#pragma once 
#include <string>

#include "robot/intake.h"
#include "robot/clamp.h"
#include "robot/doinker.h"
#include "robot/lift.h"


namespace Robot {

    class Autonomous{
        public:
            enum routine { redLeft = 1, redRight = 2, redLeftAWP = 3, redLeave = 4, blueLeft = -1, blueRight = -2, blueRightAWP = -3, blueLeave = -4, skills = 0};
            
            static routine auton;

            static std::string autonName;

            static std::string allianceColor;

            void autonMove(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift);

            static void autonSwitcher(int autonNum);

        private:
            void auton1(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift);

            void auton2(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift);
            
            void auton3(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift);
            
            void auton4(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift);
            
            void auton5(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift);

            void auton6(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift);

            void auton7(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift);

            void auton8(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift);

            void auton9(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift);

    };

}