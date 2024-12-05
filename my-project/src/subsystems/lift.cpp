#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
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
    0,    // Empty
    20,   // Loading 1
    145,  // Wallstake
    200   // Down / Wallstake

};

size_t liftIndex(0);

lemlib::PID liftPID(5, 0, 0);

void Lift::init() {
    liftMotor.set_brake_mode(pros::MotorBrake::hold);
    liftMotor.set_encoder_units(pros::MotorUnits::degrees);
    liftMotor.move_relative(-10, 100);
    pros::delay(500);
    liftMotor.tare_position();
}


void Lift::run() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
        liftIndex++; //increase index by 1
        liftIndex = std::min(liftIndex, size_t(positions.size() - 1)); //keep w/in bounds
        liftMotor.move_absolute(positions[liftIndex], 100); //move arm

    }
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
        liftIndex--; //decrease index by 1
        liftIndex = std::max(liftIndex, size_t(0)); //keep w/in bounds
        liftMotor.move_absolute(positions[liftIndex], 100); //move arm
    }
}

void Lift::setPosition(int newIndex){
    //update lift position/index
   liftIndex = newIndex;
}



pros::Task liftTask(
    [](){
        while (true){
            double out = liftPID.update(positions[liftIndex] - pot.get_angle() / 100.0);
            liftMotor.move_voltage(out * 100);
            pros::delay(10); //save resources
        }
    }
);

