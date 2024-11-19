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

void Intake::run(std::string allianceColor) {
    allianceColor = "red";
    //set the state of the conveyor
    int conveyorState = 0;
    //variables for hue ranges of rings!
    double redLower = 0;
    double redUpper = 40;
    double blueLower = 150;
    double blueUpper = 270;
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        intakeMotor.move_velocity(600); //spin motors at full speed ^-^
        conveyorMotor.move_velocity(600);
        controller.print(1,0, std::to_string(distance.get_distance()).c_str()); 
    
        if ((conveyorState == 0 && optical.get_proximity() > 180)){ //check if there's an object close to the sensor
        if (((allianceColor == std::string("red") && blueLower < optical.get_hue())) || //check if the object close is blue (if we're red) or red (if we're blue)
            ((allianceColor == std::string("blue") && redLower < optical.get_hue()))){
                conveyorState = 1; //if the object is of the opposite color, trigger the state change
        }           
        }

        switch (conveyorState){
            case 0:
            //do nothing
            break;
            case 1:
            if (distance.get_distance() < 4){
            //NEED TO TUNE: make ring fly off conveyor, switch back to normal state
            pros::delay(30);
            conveyorMotor.move_velocity((0));
            pros::delay(70);
            conveyorMotor.move_velocity((-600));
            pros::delay(70);
            conveyorMotor.move_velocity((0));
            conveyorState = 2;}
            break;
            case 2:
            //change state back to normal to rerun process
            conveyorState = 0;
            break;
        }
        
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        intakeMotor.move_velocity(-600);
        conveyorMotor.move_velocity(-600);
    }
    else{
        intakeMotor.move_velocity(0);
        conveyorMotor.move_velocity(0);
    }
}

void Intake::autoRun(int direction, int speed, std::string allianceColor){
    int conveyorState = 0;
    int redLower = 10;
    int redUpper = 40;
    int blueLower = 170;
    int blueUpper = 240;
/*
    while (true){
        if ((conveyorState == 0) && (optical.get_proximity() > 180)){
            if (((allianceColor == "red") && (blueLower < optical.get_hue() < blueUpper)) || 
            ((allianceColor == "blue") && (redLower < optical.get_hue() < redUpper))){
                conveyorState = 1;
        }           
        }
        switch (conveyorState){
            case 0:
            //run conveyor like normal
            intakeMotor.move_velocity((direction * speed));
            conveyorMotor.move_velocity((direction * speed));
            break;
            case 1:
            pros::delay(300);
            intakeMotor.move_velocity((0));
            conveyorMotor.move_velocity((0));
            conveyorState = 2;
            break;
            case 2:
            conveyorState = 0;
            break;
        }
        pros::delay(20);
    }*/
    


}

void Intake::stop(){
    intakeMotor.move_velocity((0));
    conveyorMotor.move_velocity((0));
}