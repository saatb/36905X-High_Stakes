#include "pros/misc.h"
#include "robot/intake.h"
#include <cassert>
#include "globals.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

using namespace Robot;
using namespace Robot::Global;
Intake::Intake() {
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

void Intake::autoRun(int direction, int speed){
    intakeMotor.move_velocity((direction * speed));
    conveyorMotor.move_velocity((direction * speed));
}

void Intake::stop(){
    intakeMotor.move_velocity((0));
    conveyorMotor.move_velocity((0));
}