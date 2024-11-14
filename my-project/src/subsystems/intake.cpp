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
    std::string allianceColor1 = "blue";
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        intakeMotor.move_velocity(600);
        conveyorMotor.move_velocity(600);

    controller.print(1,0, std::to_string(optical.get_hue()).c_str());
    int conveyorState = 0;
    double redLower = 0;
    double redUpper = 30;
    double blueLower = 150;
    double blueUpper = 270;

    //while (true){
        if ((conveyorState == 0) && (optical.get_proximity() > 180)){
        if (((allianceColor1 == "red") && (blueLower < optical.get_hue() < blueUpper)) || 
            ((allianceColor1 == "blue") && (redLower < optical.get_hue() < redUpper))){
                conveyorState = 1;
        }           
        }

        switch (conveyorState){
            case 0:
            //do nothing
            break;
            case 1:
            pros::delay(50);
            intakeMotor.move_velocity((0));
            conveyorMotor.move_velocity((0));
            conveyorState = 2;
            break;
            case 2:
            conveyorState = 0;
            break;
        }
        //pros::delay(20);*/
    //}
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

void Intake::autoRun(int direction, int speed){
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