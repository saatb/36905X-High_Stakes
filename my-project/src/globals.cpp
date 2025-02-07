#include "globals.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

namespace Robot{
namespace Global{

    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    //drive motor groups
    pros::MotorGroup driveRight({14,15,16}, pros::MotorGearset::blue, pros::v5::MotorUnits::degrees);
    pros::MotorGroup driveLeft({-13,-19,-20}, pros::MotorGearset::blue, pros::v5::MotorUnits::degrees);

    //intake and conveyor motors
    pros::Motor intakeMotor(12, pros::v5::MotorGears::green, pros::v5::MotorUnits::deg);
    pros::Motor conveyorMotor(-9, pros::v5::MotorGears::blue, pros::v5::MotorUnits::deg);
    pros::Motor liftMotor(-6, pros::v5::MotorGears::green, pros::v5::MotorUnits::deg);

    //sensors
    pros::Optical optical(8);
    pros::Distance distance(10);
    //pros::adi::Potentiometer pot('D');
    pros::Rotation rotation(7);

    //imu
    pros::Imu imu(11);

    //odom sensors
    pros::Rotation horSensor(-1);
    pros::Rotation verSensor(-2);

    //tracking wheels -.5, -3.33
    lemlib::TrackingWheel hor(&horSensor, lemlib::Omniwheel::NEW_275, -4);
    lemlib::TrackingWheel ver(&verSensor, lemlib::Omniwheel::NEW_275, -.5);

    //make drivetrain
    lemlib::Drivetrain drivetrain(&driveLeft, &driveRight, 
	12, lemlib::Omniwheel::NEW_325, 450, 2);

    //odom (imu, 1x horizontal, 1x vertical)
    lemlib::OdomSensors sensors(&ver, nullptr, 
	&hor, nullptr, &imu);

    //PID config
    // lateral PID controller
    /*
    lemlib::ControllerSettings lateralController(10, // proportional gain (kP)
                                                0, // integral gain (kI)
                                                3, // derivative gain (kD)
                                                3, // anti windup
                                                1, // small error range, in inches
                                                100, // small error range timeout, in milliseconds
                                                3, // large error range, in inches
                                                500, // large error range timeout, in milliseconds
                                                20 // maximum acceleration (slew)
    );*/

    lemlib::ControllerSettings lateralController(15, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

    // angular PID controller
    lemlib::ControllerSettings angularController(5.5, // proportional gain (kP) 5
                                                0, // integral gain (kI)
                                                50, // derivative gain (kD)
                                                3, // anti windup
                                                1, // small error range, in degrees
                                                100, // small error range timeout, in milliseconds
                                                3, // large error range, in degrees
                                                500, // large error range timeout, in milliseconds
                                                0 // maximum acceleration (slew)
    );

    lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

    
    //pneumatics
    pros::adi::Pneumatics clampControl('H', false);
    pros::adi::Pneumatics doinker('G', false);
}
}