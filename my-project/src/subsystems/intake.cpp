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

bool autoSortEnabled(0);
bool antiStallEnabled(0);

void enableAutoSort()
{
          // Enable auto sort
}

void disableAutoSort()
{
         // Disable auto sort
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
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
        if (autoSortEnabled){
            optical.set_led_pwm(0);
            autoSortEnabled = false;
        }
        else {
            optical.set_led_pwm(60);  // turn on light for color sort (on by default)
            autoSortEnabled = true;  
        }
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

void sort()
{
controller.print(1, 1, "ring detected!");
pros::delay(150);
conveyorMotor.move_velocity((0));
pros::delay(150);
conveyorMotor.move_velocity((-600));
//pros::delay(150);
//conveyorMotor.move_velocity((0));
}

void antiStall(int goal)
{
controller.print(1, 1, "stall!");
conveyorMotor.move_velocity(goal * -1);
pros::delay(1000);
}

pros::Task colorSortingTask(
    [](){
        double redUpper = 30;
        double blueLower = 100;
        while (true) {
            if (autoSortEnabled){
        
        std::string allianceColor = Robot::Autonomous::allianceColor;
        	if (((allianceColor == std::string("red") && blueLower < optical.get_hue())) || //check if the object close is blue (if we're red) or red (if we're blue)
            	((allianceColor == std::string("blue") && redUpper > optical.get_hue()))){
                //if the object is of the opposite color, trigger the conveyor to stop
                //double startTime = pros::millis();
                //start a timer
                //while ((pros::millis() - startTime) < 300){
                sort();
                }
        }          
		pros::delay(20); //save resources
        }
    }
);

pros::Task antiStallTask(
    [](){
        
        while (true) {
            if (antiStallEnabled){ //only run code if anti stall is enabled
                double goal = conveyorMotor.get_target_velocity();
                double actual = conveyorMotor.get_actual_velocity();
        	if ((actual > -100 && goal == -600) || //check if the actual velocity of the conveyor is close to 0, only if conveyor should be running
            	(actual < 100 && goal == 600)) {
                controller.print(1, 1, "stall!");
                conveyorMotor.move_velocity(goal * -1);
                pros::delay(1000);
                }
            else {
                controller.clear_line(1);
            }
        }          
		pros::delay(20); //save resources
        }
    }
);