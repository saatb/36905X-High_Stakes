#pragma once
#include "api.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/misc/lv_area.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"

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
    extern pros::Motor liftMotor;

    extern pros::MotorGroup driveLeft;
    extern pros::MotorGroup driveRight;

    extern lemlib::Drivetrain drivetrain;
    extern lemlib::OdomSensors sensors;

    // moving PID
    extern lemlib::ControllerSettings lateralController;

    // turning PID
    extern lemlib::ControllerSettings angularController;

    extern lemlib::Chassis chassis;

    //pneumatics
    extern pros::adi::Pneumatics clampControl;
    extern pros::adi::Pneumatics doinker;

    //sensors
    extern pros::Optical optical;
    extern pros::Distance distance;
    extern pros::adi::Potentiometer pot;
}}