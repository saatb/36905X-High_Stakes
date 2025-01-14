#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rotation.h"
#include "robot/intake.h"
#include <cassert>
#include <cstddef>
#include <string>
#include "globals.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "robot/auton.h"

using namespace Robot;
using namespace Robot::Global;
Lift::Lift() {
}

std::vector<double> positions = {
    5,    // Empty
    25,   // Loading
    80,   //up, not score (rest)
    150  // bring to wall stake 150

};

//positions are based on ROTATION SENSOR readings (degrees), NOT motor

size_t liftIndex(0);

lemlib::PID liftPID(2, 0, 0);

void Lift::init() {
    liftMotor.set_brake_mode(pros::MotorBrake::hold);
    liftMotor.set_encoder_units(pros::MotorUnits::degrees);
    pros::delay(500);
    liftMotor.tare_position();
    //srotation.reset_position();
}


void Lift::run() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
        liftIndex = liftIndex + 1; //increase index by 1
        if (liftIndex == 2){
            liftIndex = liftIndex + 1;
        }
        liftIndex = std::min(liftIndex, size_t(positions.size() - 1)); //keep w/in bounds
        liftMotor.move_absolute(positions[liftIndex], 200); //move arm

    }
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        liftIndex = liftIndex - 1; //decrease index by 1
        if (liftIndex == 2){
            liftIndex = liftIndex - 1;
        }
        liftIndex = std::max(liftIndex, size_t(0)); //keep w/in bounds
        liftMotor.move_absolute(positions[liftIndex], -200); //move arm
    }
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
        setPosition(2);
    }
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
        setPosition(0);
    }
    /*
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
        //zero the lift manually
        liftMotor.move_relative(-10, 200);
        pros::delay(500);
        liftMotor.tare_position();
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
        //manual control of lady brown
        liftMotor.move_velocity(20);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
        //manual control of lady brown
        liftMotor.move_velocity(-20);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
        //manual control of lady brown
        liftMotor.move_velocity(0);
    }*/
}

void Lift::setPosition(int newIndex){
    //update lift position/index
   liftIndex = newIndex;
}

/*
void Lift::autoRun(int position){
    liftMotor.move_absolute(positions[position], 200); //move arm
}
*/

pros::Task liftTask(
    [](){
        while (true){
            double out = liftPID.update(positions[liftIndex] - (rotation.get_angle() / 100.0));
            liftMotor.move_voltage(out * 100);
            //controller.print(0, 0, "%f, %f", rotation.get_angle(), liftMotor.get_position());
            pros::delay(10); //save resources
        
        }
    }
);