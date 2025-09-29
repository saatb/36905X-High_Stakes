#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/widgets/lv_img.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/device.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include <chrono>
#include <cstdio>

#include "globals.h"
#include "pros/apix.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "robot/drivetrain.h"
#include "robot/auton.h"
#include "robot/intake.h"
#include "robot/clamp.h"
#include "robot/doinker.h"
#include "robot/lift.h"
#include "screen/selector.h"

using namespace Robot;
using namespace Robot::Global;

struct RobotSubsystems {
   Robot::Autonomous autonomous;
   Robot::Drivetrain drivetrain;
   Robot::Intake intake;
   Robot::Clamp clamp;
   Robot::Doinker doinker;
   Robot::Lift lift;
} subsystem;

struct RobotScreen{
	Robot::autonSelectorScreen selector;
} screen;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	//calibrate chassis
    chassis.calibrate();

	screen.selector.selector();

	//zero chassis pose
	chassis.setPose(0,0,0);

	//set drive motors to brake
	driveRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	subsystem.lift.init();//init lift
	//config optical sensor
	optical.set_integration_time(40);
	optical.set_led_pwm(100);
/*
	pros::lcd::initialize();
	pros::Task screen_task([&](){
  
		while (true){
			pros::lcd::print(0, "X: %f", chassis.getPose().x);
			pros::lcd::print(1, "Y: %f", chassis.getPose().y);
			pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
			pros::lcd::print(3, "Conveyor: %d", conveyorMotor.get_current_draw());
			pros::lcd::print(4, "Color: %f", optical.get_hue());
			pros::lcd::print(5, "Distance: %d", distance.get());
			pros::delay(40);
		}
	});*/
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	screen.selector.selector();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	screen.selector.selector();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	subsystem.autonomous.autonMove(subsystem.intake, subsystem.clamp, subsystem.doinker, subsystem.lift);
}

void skillsMacro(){
   subsystem.lift.setPosition(1);
   subsystem.intake.disableAntiStall();
   subsystem.intake.autoRun();
   pros::delay(500);
   subsystem.intake.stop();
   subsystem.lift.setPosition(4);
   pros::delay(1000);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */


void opcontrol() {
	while (true){
		subsystem.drivetrain.run();
		subsystem.intake.run();
		subsystem.clamp.run();
		subsystem.doinker.run();
		subsystem.lift.run();
		pros::delay(20); //save resources
	}
}

