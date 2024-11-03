#include "pros/misc.h"
#include "robot/intake.h"
#include <cassert>
#include "globals.h"

using namespace Robot;
using namespace Robot::Global;
Intake::Intake() {
   controller.print(0, 0, "Intake initialized");
}

void Intake::run() {
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        intakeMotor.move_velocity(600);
        conveyorMotor.move_velocity(600);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        intakeMotor.move_velocity(-600);
        conveyorMotor.move_velocity(-600);
    }
    else{
        intakeMotor.brake();
        conveyorMotor.brake();
    }
}
