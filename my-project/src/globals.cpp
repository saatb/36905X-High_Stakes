#include "globals.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"

namespace Robot{
namespace Global{

    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    //intake and conveyor motors
    pros::Motor intakeMotor(12, pros::v5::MotorGears::blue, pros::v5::MotorUnits::deg);
    pros::Motor conveyorMotor(13, pros::v5::MotorGears::blue, pros::v5::MotorUnits::deg);

    //pneumatic (singular for now)
    pros::adi::Pneumatics clampControl('C', false);

    //imu
    pros::Imu imu(7);

    //drive motor groups
    pros::MotorGroup rightMotors({1,2,3}, pros::MotorGearset::blue, pros::v5::MotorUnits::degrees);
    pros::MotorGroup leftMotors({-4,-5,-6}, pros::MotorGearset::blue, pros::v5::MotorUnits::degrees);

    //make drivetrain
    lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 
	12, lemlib::Omniwheel::NEW_325, 450, 2);

    //odom (only IMU!)
    lemlib::OdomSensors sensors(nullptr, nullptr, 
	nullptr, nullptr, &imu);

    //PID config
    // lateral PID controller
    lemlib::ControllerSettings lateralController(10, // proportional gain (kP)
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
    lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                                0, // integral gain (kI)
                                                10, // derivative gain (kD)
                                                3, // anti windup
                                                1, // small error range, in degrees
                                                100, // small error range timeout, in milliseconds
                                                3, // large error range, in degrees
                                                500, // large error range timeout, in milliseconds
                                                0 // maximum acceleration (slew)
    );

    lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);
}
}