#pragma once
#include "api.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/misc/lv_area.h"
#include "pros/motors.hpp"

namespace Robot{

namespace Global {
    extern pros::Controller controller;
    extern pros::Motor rightFront;
    extern pros::Motor rightMid;
    extern pros::Motor rightBack;
    extern pros::Motor leftFront;
    extern pros::Motor leftMid;
    extern pros::Motor leftRight;

    extern pros::Motor intakeMotor;
    extern pros::Motor conveyorMotor;

    extern pros::MotorGroup driveLeft;
    extern pros::MotorGroup driveRight;

    extern lemlib::Drivetrain drivetrain;
    extern lemlib::OdomSensors sensors;

    // moving PID
    extern lemlib::ControllerSettings lateral_controller;

    // turning PID
    extern lemlib::ControllerSettings angular_controller;

    extern lemlib::Chassis chassis;

    extern pros::adi::Pneumatics clampControl;
}}