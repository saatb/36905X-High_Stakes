#include "robot/auton.h"
#include "globals.h"
#include "main.h" // IWYU pragma: export
#include "pros/rtos.hpp"
#include "robot/clamp.h"
#include "robot/drivetrain.h"
#include <sys/types.h>
#include <time.h>
#include "lemlib/timer.hpp"
#include "robot/intake.h"

using namespace Robot;
using namespace Robot::Global;
using namespace lemlib;


Autonomous::routine Autonomous::auton = blueRight;
std::string				  Autonomous::autonName;

/*
NOTES:
   - imu is mounted horizontally, so x and y are FLIPPED from path.jerryio
   - towards negative corner is NEGATIVE x, toward positive corner is POSITIVE x (opposite of path.jerryio)
   - towards blue alliance is POSITIVE y, towards red alliance is NEGATIVE y (same as path.jerryio)
*/

void Autonomous::auton1(Intake &intake, Clamp &clamp){
    // autonomous 1 --> redLeft, blueRight (single mogo) DON'T KNOW IF WORKS ON BOTH SIDES

    //line up straight back from mogo
    //picks up mogo, then scores preload + adjacent ring, then scores both midline rings
    //then touches ladder (most likely, zipties)

    //release first stage
    
    intake.autoRun(-1, 600);
    pros::delay(300);
    intake.stop();

    //set pose
    chassis.setPose(22.946, -60.144, 180 );

    //move to mogo, motion chained
    chassis.moveToPoint(22.946, -45, 5000, {.forwards = false, .minSpeed = 63, .earlyExitRange = 4});
    chassis.moveToPoint(22.946, -32.919, 5000, {.forwards = false, .maxSpeed = 63});
    pros::delay(500);

    //clamp
    clamp.toggle();
    pros::delay(300);

    //start scoring rings
    intake.autoRun(1, 600);

    //move to single stack adjacent to mogo 
    chassis.moveToPoint(7.296, -32.919,5000); // x-coord from jerry was 38.596

    //slowly move to leftmost stack of rings on midline
    chassis.moveToPose(-7.296, -18.919, 0, 3500);

    //back up to prepare for second stack of rings
    chassis.moveToPoint(-8.296, -30.919, 5000, {.forwards = false});

    //move to second stack
    chassis.moveToPose(-2.296, -12.919, 30, 3500); //{.maxspeed = 35}

    //back up after scoring
    chassis.moveToPoint(-8.296, -24.919, 5000, {.forwards = false});
    pros::delay(1000);
    
    //go to ladder
    chassis.moveToPose(40.5, -32.919, 40, 5000);
}

void Autonomous::auton2(Intake &intake, Clamp &clamp){
   //TODO: make autonomous 2 --> mogo 
}

void Autonomous::auton3(Intake &intake, Clamp &clamp){
   //TODO: make autonomous 3
}

void Autonomous::auton4(Intake &intake, Clamp &clamp){
   //TODO: make autonomous 4
}

void Autonomous::auton5(Intake &intake, Clamp &clamp){
   //TODO: make autonomous 5
}


void Autonomous::autonMove(Intake &intake, Clamp &clamp){
    auton1(intake, clamp);
}

void Autonomous::autonSwitcher(int autonNum)
{
	switch (autonNum) {
	case 1:
		Autonomous::autonName = "Red Left";
		Autonomous::auton	  = redLeft;
		break;
	case 2:
		Autonomous::autonName = "Red Right";
		Autonomous::auton	  = redRight;
		break;
	case -1:
		Autonomous::autonName = "Blue Left";
		Autonomous::auton	  = blueLeft;
		break;
	case -2:
		Autonomous::autonName = "Blue Right";
		Autonomous::auton	  = blueRight;
		break;
	}
	std::cout << "Current auton: " + Autonomous::autonName << std::endl;
}