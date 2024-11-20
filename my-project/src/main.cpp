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
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "robot/drivetrain.h"
#include "robot/auton.h"
#include "robot/intake.h"
#include "robot/clamp.h"
#include "screen/selector.h"

LV_IMG_DECLARE(test1);
LV_IMG_DECLARE(test2);
LV_IMG_DECLARE(test3);
LV_IMG_DECLARE(test4);
LV_IMG_DECLARE(test5);
LV_IMG_DECLARE(test6);
LV_IMG_DECLARE(test7);
LV_IMG_DECLARE(test8);

using namespace Robot;
using namespace Robot::Global;

struct RobotSubsystems {
   Robot::Autonomous autonomous;
   Robot::Drivetrain drivetrain;
   Robot::Intake intake;
   Robot::Clamp clamp;
} subsystem;

struct RobotScreen{
	Robot::autonSelectorScreen selector;
} screen;

void myTask(){
	std::string allianceColor = Autonomous::allianceColor;
	int conveyorState = 0;
    double redUpper = 40;
    double blueLower = 150;
	while (true){
		if ((conveyorState == 0 && optical.get_proximity() > 180)){ //check if there's an object close to the sensor
        	if (((allianceColor == std::string("red") && blueLower < optical.get_hue())) || //check if the object close is blue (if we're red) or red (if we're blue)
            	((allianceColor == std::string("blue") && redUpper > optical.get_hue()))){
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
		pros::delay(10);
}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	if (pros::c::get_plugged_type(10) == pros::c::E_DEVICE_IMU) {
      chassis.calibrate();
   }

	chassis.setPose(0,0,0);
	driveRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	optical.disable_gesture();
	optical.set_led_pwm(25);

	pros::Task::create(myTask, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "color sorting");

	//screen.selector.selector();
	/*pros::Task screen_task([&](){
		while (true){
			pros::lcd::print(0, "X: %f", chassis.getPose().x);
			pros::lcd::print(1, "Y: %f", chassis.getPose().y);
			pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
			pros::delay(20);
		}
	});*/
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	/*lv_obj_t *img = lv_img_create(lv_scr_act());
	lv_img_set_src(img, &test2);
	lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);*/
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
	/*lv_obj_t *img = lv_img_create(lv_scr_act());
	lv_img_set_src(img, &test1);
	lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);*/
	subsystem.autonomous.autonMove(subsystem.intake, subsystem.clamp);
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
	/*lv_obj_t *img = lv_img_create(lv_scr_act());
	lv_img_set_src(img, &test3);
	lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);*/
	//pros::Task::create(myTask, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "color sorting");
	while (true){
		subsystem.drivetrain.run();
		subsystem.intake.run(Autonomous::allianceColor);
		subsystem.clamp.run();
		pros::delay(20);
	}
}

