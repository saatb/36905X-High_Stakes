#include "robot/drivetrain.h"

#include "globals.h"

using namespace Robot;
using namespace Robot::Global;

Drivetrain::DRIVE_MODE Drivetrain::driveMode = TANK_DRIVE;

Drivetrain::Drivetrain()
{
	Drivetrain::driveMode = TANK_DRIVE;
}

void Drivetrain::run(){
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    chassis.tank(leftY, rightY);
    //controller.print(0, 0, "L: %d R: %d       ", leftY, rightY);
}