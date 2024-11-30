#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "robot/intake.h"
#include <cassert>
#include <string>
#include "globals.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
using namespace Robot;
using namespace Robot::Global;
Intake::Intake() {
}

void Intake::run() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        //spin motors at full speed ^-^
        intakeMotor.move_velocity(600); 
        conveyorMotor.move_velocity(600);        
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        //spin motors at full speed but backwards :3
        intakeMotor.move_velocity(-600);
        conveyorMotor.move_velocity(-600);
    }
    else{
        //die.
        intakeMotor.move_velocity(0);
        conveyorMotor.move_velocity(0);
    }
}

void Intake::autoRun(int direction, int intakeSpeed, int conveyorSpeed){
    //spins motors based on direction (-1 and 1) and speed (600 for conveyor, 200 for intake)
    intakeMotor.move_velocity((direction * intakeSpeed));
    conveyorMotor.move_velocity((direction * conveyorSpeed));
}

void Intake::stop(){
    //die.
    intakeMotor.move_velocity((0));
    conveyorMotor.move_velocity((0));
}

pros::Task colorSortingTask(
    []{
        while (true) {
            //
        }
    }
);