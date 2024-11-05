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

bool quals = false;
bool goalRush = false;

/*
NOTES:
   - imu is mounted horizontally, so x and y are FLIPPED from path.jerryio
   - towards negative corner is NEGATIVE x, toward positive corner is POSITIVE x (opposite of path.jerryio)
   - towards blue alliance is POSITIVE y, towards red alliance is NEGATIVE y (same as path.jerryio)
   - RED to BLUE transformation is inverting y-values (might also need to change headings)
*/

void Autonomous::auton1(Intake &intake, Clamp &clamp){
    // autonomous 1 --> blueRight (single mogo)
    // CHANGED FROM redLeft by inverting y-values (neg to pos)

    //line up straight back from mogo
    //picks up mogo, then scores preload + adjacent ring, then scores both midline rings
    //then touches ladder with intake
}

void Autonomous::auton2(Intake &intake, Clamp &clamp){
   // autonomous 2 --> redLeft (single mogo)

    //line up straight back from mogo
    //picks up mogo, then scores preload + adjacent ring, then scores both midline rings
    //then touches ladder with intake

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

    //clamp mogo
    clamp.toggle();
    pros::delay(300);

    //start scoring rings
    intake.autoRun(1, 600);

    //move to single stack adjacent to mogo 
    chassis.moveToPoint(7.296, -32.919,5000); // x-coord from jerry was 38.596

    //slowly move to leftmost stack of rings on midline
    chassis.moveToPose(-7.296, -18.919, 10, 3000);

    //back up to prepare for second stack of rings
    chassis.moveToPoint(0, -30.919, 5000, {.forwards = false});

    //move to second stack
    chassis.moveToPose(-2.296, -12.919, 0, 3500); //{.maxspeed = 35}

    //back up after scoring
    chassis.moveToPoint(-8.296, -24.919, 5000, {.forwards = false});

    //allow time for conveyor to score
    pros::delay(1000);

    //stop intake, keep conveyor going to score ring if needed
    intakeMotor.move_velocity(0);

    if (quals){
    //QUALS CODE
    //go to ladder and touch with intake
    chassis.moveToPose(40.5, -32.919, 40, 5000);}

    else {
    //ELIMS CODE
    //get as close to positive corner as possible with back of robot
    chassis.moveToPoint(84.95, -40.919, 5000, {.forwards = false});}
}

void Autonomous::auton3(Intake &intake, Clamp &clamp){
   //TODO: make autonomous 3 redRight -->

   //release first stage
   intake.autoRun(-1, 600);
   pros::delay(300);
   intake.stop();

   if (goalRush == false){
   //set pose
   chassis.setPose(22.946, -60.144, 180 );

   //move to mogo, motion chained
   chassis.moveToPoint(-22.946, -45, 5000, {.forwards = false, .minSpeed = 63, .earlyExitRange = 4});
   chassis.moveToPoint(-22.946, -32.919, 5000, {.forwards = false, .maxSpeed = 63});
   pros::delay(500);

   }

   else {
   //set pose
   chassis.setPose(-35.804, -58.943, 180);

   }
}

void Autonomous::auton4(Intake &intake, Clamp &clamp){
   //TODO: make autonomous 4 blueLeft --> 
}

void Autonomous::auton5(Intake &intake, Clamp &clamp){
   //TODO: make autonomous 5 --> skills
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