#include "pros/misc.h"
#include "robot/intake.h"
#include <cassert>
#include "globals.h"

using namespace Robot;
using namespace Robot::Global;

void Intake::run() {
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        intakeMotor.move_velocity(600);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        conveyorMotor.move_velocity(600);
    }
    else{
        intakeMotor.brake();
        conveyorMotor.brake();
    }
}
