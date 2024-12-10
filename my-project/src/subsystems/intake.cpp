#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "robot/intake.h"
#include <cassert>
#include <string>
#include "globals.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "robot/auton.h"
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

void Intake::wallstake(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        conveyorMotor.move_relative(-20, -600);
    }
}

pros::Task colorSortingTask(
    [](){
        double redUpper = 40;
        double blueLower = 150;
        while (true) {
        
        std::string allianceColor = Robot::Autonomous::allianceColor;
		if ((optical.get_proximity() > 180)){ //check if there's an object close to the sensor
        	if (((allianceColor == std::string("red") && blueLower < optical.get_hue())) || //check if the object close is blue (if we're red) or red (if we're blue)
            	((allianceColor == std::string("blue") && redUpper > optical.get_hue()))){
                //if the object is of the opposite color, trigger the conveyor to stop
                //double startTime = pros::millis();
                //start a timer
                //while ((pros::millis() - startTime) < 300){
                
                conveyorMotor.move_velocity((0));
                controller.print(1, 1, "ring detected!");
                pros::delay(150);
                }
        }          
		pros::delay(20); //save resources
        }
    }//}
);