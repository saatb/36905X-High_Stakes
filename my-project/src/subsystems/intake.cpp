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
bool antiStallEnabled(1);
bool detectedRing(0);

void Intake::enableAutoSort()
{
          // Enable auto sort
          autoSortEnabled = true;
}

void Intake::disableAutoSort()
{
         // Disable auto sort
         autoSortEnabled = false;
}

void Intake::enableAntiStall()
{
          // Enable auto sort
          antiStallEnabled = true;
}

void Intake::disableAntiStall()
{
         // Disable auto sort
         antiStallEnabled = false;
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
            autoSortEnabled = false;
        }
        else {
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
if (distance.get_distance() < 20){
controller.print(1, 1, "ring detected!");
pros::delay(150);
conveyorMotor.move_velocity((0));
pros::delay(300);
conveyorMotor.move_velocity((-600));
detectedRing = false;
}
//pros::delay(150);
//conveyorMotor.move_velocity((0));
}

void antiStall()
{
controller.print(1, 1, "stall!");
conveyorMotor.move_velocity(-600);
intakeMotor.move_velocity(-200);
pros::delay(200);
conveyorMotor.move_velocity(600);
intakeMotor.move_velocity(600);
}

pros::Task colorSortingTask(
    [](){
        double redUpper = 30;
        double blueLower = 100;
        while (true) {
            if ((autoSortEnabled) && (!detectedRing)){
        
        std::string allianceColor = Robot::Autonomous::allianceColor;
        	if (((allianceColor == std::string("red") && blueLower < optical.get_hue())) || //check if the object close is blue (if we're red) or red (if we're blue)
            	((allianceColor == std::string("blue") && redUpper > optical.get_hue()))){
                //if the object is of the opposite color, trigger the conveyor to stop
                //double startTime = pros::millis();
                //start a timer
                //while ((pros::millis() - startTime) < 300){
                detectedRing = true;
                sort();
                }
        }
        if (detectedRing){
            sort();
        }   

        controller.print(1, 1, "%f, %f", distance.get(), optical.get_hue());       
		pros::delay(20); //save resources
        }
    }
);

pros::Task antiStallTask(
    [](){
        double timer1 = 0;
        while (true) {
            if (antiStallEnabled){ //only run code if anti stall is enabled
                double convGoal = conveyorMotor.get_target_velocity();
                double intakeGoal = intakeMotor.get_target_velocity();
                double draw = conveyorMotor.get_current_draw();
                
        	if (draw > 2450) {
                    if (timer1 > 40){
                        antiStall();
                        timer1 = 0;
                    }
                    else {
                        timer1 += 20;
                    }
                    }
            else {
                timer1 = 0;
            }
        }
		pros::delay(20); //save resources
        }
    }
);