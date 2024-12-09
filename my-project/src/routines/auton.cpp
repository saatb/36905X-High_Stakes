#include "robot/auton.h"
#include "globals.h"
#include "lemlib/chassis/chassis.hpp"
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


Autonomous::routine Autonomous::auton = redLeft;
std::string				  Autonomous::autonName;
std::string Autonomous::allianceColor = "red";

bool quals = true;
bool goalRush = false;

/*
NOTES:
   - imu is mounted horizontally, so x and y are FLIPPED from path.jerryio
   - towards negative corner is NEGATIVE x, toward positive corner is POSITIVE x (opposite of path.jerryio)
   - towards blue alliance is POSITIVE y, towards red alliance is NEGATIVE y (same as path.jerryio)
   - RED to BLUE transformation is inverting y-values (might also need to change headings)
*/

void Autonomous::auton1(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
   controller.print(0, 0, "br, q: %d", quals);
    // autonomous 1 --> blueRight (single mogo)
    // CHANGED FROM redLeft by inverting y-values (neg to pos) and theta - 180

    //line up straight back from mogo
    //picks up mogo, then scores preload + adjacent ring, then scores both midline rings
    //then touches ladder with intake

    //release first stage
    intake.autoRun(-1, 600);
    pros::delay(150);
    intake.stop();

    //set pose
    chassis.setPose(22.946, 60.144, 0 );

    //move to mogo, motion chained
    chassis.moveToPoint(22.946, 45, 5000, {.forwards = false, .minSpeed = 63, .earlyExitRange = 4});
    chassis.moveToPoint(22.946, 32.919, 5000, {.forwards = false, .maxSpeed = 63});
    pros::delay(500);

    //clamp mogo
    clamp.toggle();
    pros::delay(300);

    //start scoring rings
    intake.autoRun(1, 600);

    //move to single stack adjacent to mogo 
    chassis.moveToPoint(7.296, 32.919,5000); // x-coord from jerry was 38.596

    //slowly move to leftmost stack of rings on midline
    chassis.moveToPose(-3.296, 15.919, 180, 3000);

    //back up to prepare for second stack of rings
    chassis.moveToPoint(3, 30.919, 2000, {.forwards = false});

    //move to second stack
    chassis.moveToPose(2.296, 15.919, (5 - 180), 3500);

    //back up after scoring
    chassis.moveToPoint(-5.296, 24.919, 5000, {.forwards = false});

    //allow time for conveyor to score
    pros::delay(1000);

    //stop intake, keep conveyor going to score ring if needed
    intakeMotor.move_velocity(0);

    if (quals == true){
    //QUALS CODE
    //go to ladder and touch with intake
    chassis.moveToPose(43.5, 32.919, (5 - 180), 5000);}

    else {
    //ELIMS CODE
    //get as close to positive corner as possible with back of robot
    chassis.moveToPoint(78.95, 40.919, 5000, {.forwards = false});}
   }

void Autonomous::auton2(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
   controller.print(0, 0, "rl, q: %d", quals);
   //DONE
   //autonomous 2 --> redLeft (single mogo)

    //line up straight back from mogo
    //picks up mogo, then scores preload + adjacent ring, then scores both midline rings
    //then touches ladder with intake

    //release first stage
    intake.autoRun(-1, 600);
    pros::delay(150);
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
    pros::delay(1000);
    chassis.moveToPoint(7.296, -37.919,5000, {.forwards = false});
    //slowly move to leftmost stack of rings on midline
    chassis.moveToPose(-6.296, -18.919, 0, 3000);

    //back up to prepare for second stack of rings
    chassis.moveToPoint(0, -30.919, 2000, {.forwards = false});

    //move to second stack
    chassis.moveToPose(-1.296, -15.919, 5, 3500);

    //back up after scoring
    chassis.moveToPoint(-8.296, -24.919, 5000, {.forwards = false});

    //allow time for conveyor to score
    pros::delay(1000);

    //stop intake, keep conveyor going to score ring if needed
    intakeMotor.move_velocity(0);

    if (quals == true){
    //QUALS CODE
    //go to ladder and touch with intake
    chassis.moveToPose(40.5, -32.919, 40, 5000);}

    else {
    //ELIMS CODE
    //get as close to positive corner as possible with back of robot
    chassis.moveToPoint(75.95, -40.919, 5000, {.forwards = false});}

}

void Autonomous::auton3(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
   controller.print(0, 0, "rr, q: %d, gr: %d", quals, goalRush);
   //autonomous 3 redRight --> goal rush or safe two ring
   // safe two ring touches ladder in quals, moves near pos corner in elims

   //release first stage
   intake.autoRun(-1, 600);
   pros::delay(150);
   intake.stop();

   if (goalRush == false){
    //set pose
    chassis.setPose(-22.946, -60.144, 180 );

    //move to mogo, motion chained
    chassis.moveToPoint(-22.946, -45, 5000, {.forwards = false, .minSpeed = 63, .earlyExitRange = 4});
    chassis.moveToPoint(-22.946, -32.919, 5000, {.forwards = false, .maxSpeed = 63});
    pros::delay(500);

    //clamp mogo
    clamp.toggle();
    pros::delay(300);

    //start scoring rings
    intake.autoRun(1, 600);

    //move to single stack adjacent to mogo 
    chassis.moveToPoint(-7.296, -32.919,5000); // x-coord from jerry was 38.596

    //allow time to score
    pros::delay(3000);

    if (quals == true){
      //turn around to drop mogo
      chassis.turnToHeading(270, 1500);
      pros::delay(500);

      //stop intake and drop mogo
      intake.stop();
      clamp.toggle();

      //go and touch ladder
      chassis.moveToPoint(-36, -32.19, 5000);
    }
    else {
    //move to pos corner
    chassis.moveToPoint(0, -32.919, 6000, {.forwards = false});
    intake.stop();
    }

   }

   else {
   //set pose
   chassis.setPose(-35.804, -58.943, 180);

   //rush goal, motion chained
   chassis.moveToPoint(-35.804, -25.348, 5000, {.forwards = false, .minSpeed = 63, .earlyExitRange = 4});
   chassis.moveToPose(-24.804, -16.398, 215, 2500, {.forwards = false, .maxSpeed = 63});
   pros::delay(1750);

   //clamp goal
   clamp.toggle();
   pros::delay(300);

   //swing to stack of rings and pick up
   chassis.swingToHeading(135, DriveSide::LEFT, 1500);
   intake.autoRun(1, 600);

   pros::delay(3000);

   //stop intake since we're done picking up rings
   intakeMotor.move_velocity(0);

   if (quals == true){
      //move behind other mogo
      chassis.moveToPose(-47.804, -46.943, 270, 5000);

      //drop mogo and stop conveyor
      intake.stop();
      clamp.toggle();

      //go and touch ladder
      chassis.moveToPoint(-60, -32.19, 5000);
   }
   else{
      //move to pos corner BACKWARDS
      chassis.moveToPoint(-11.804, -58.943, 5000, {.forwards = false});

      //stop intake
      intake.stop();
   }
   }
}

void Autonomous::auton4(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
controller.print(0, 0, "bl, q: %d, gr: %d", quals, goalRush);

   //TODO: make autonomous 4 blueLeft --> goal rush or safe two ring
   // safe two ring touches ladder in quals, moves near pos corner in elims
   //CHANGED from redRight by inverting y-values (neg to pos) and theta - 180 for commands

   //release first stage
   intake.autoRun(-1, 600);
   pros::delay(150);
   intake.stop();

   if (goalRush == false){
    //set pose
    chassis.setPose(-22.946, 60.144, 0 );

    //move to mogo, motion chained
    chassis.moveToPoint(-22.946, 45, 5000, {.forwards = false, .minSpeed = 63, .earlyExitRange = 4});
    chassis.moveToPoint(-22.946, 32.919, 5000, {.forwards = false, .maxSpeed = 63});
    pros::delay(500);

    //clamp mogo
    clamp.toggle();
    pros::delay(300);

    //start scoring rings
    intake.autoRun(1, 600);

    //move to single stack adjacent to mogo 
    chassis.moveToPoint(-7.296, 32.919,5000); // x-coord from jerry was 38.596

    //allow time to score
    pros::delay(3000);

    if (quals == true){
      //turn around to drop mogo
      chassis.turnToHeading((180 - 270), 1500);

      //stop intake and drop mogo
      intake.stop();
      clamp.toggle();

      //go and touch ladder
      chassis.moveToPose(-38, 32.19, 45 - 180, 5000);
    }
    else {
    //move to pos corner
    chassis.moveToPoint(0, 32.919, 6000, {.forwards = false});
    intake.stop();
    }

   }

   else {
   //set pose
   chassis.setPose(-35.804, 58.943, 0);

   //rush goal, motion chained
   chassis.moveToPoint(-35.804, 25.348, 5000, {.forwards = false, .minSpeed = 63, .earlyExitRange = 4});
   chassis.moveToPose(-24.804, 14.398, (180 - 215), 2500, {.forwards = false, .maxSpeed = 63});
   pros::delay(1750);

   //clamp goal
   clamp.toggle();
   pros::delay(300);

   //swing to stack of rings and pick up
   chassis.swingToHeading((180-135), DriveSide::RIGHT, 1500);
   intake.autoRun(1, 600);

   pros::delay(3000);

   //stop intake since we're done picking up rings
   intakeMotor.move_velocity(0);

   if (quals == true){
      //move behind other mogo
      chassis.moveToPose(-47.804, 46.943, (180 - 270), 5000);

      //drop mogo and stop conveyor
      intake.stop();
      clamp.toggle();

      //go and touch ladder
      chassis.moveToPose(-59, 36.19, 45 - 180,5000);
   }
   else if (quals == false){
      //move to pos corner BACKWARDS
      chassis.moveToPoint(-11.804, 58.943, 5000, {.forwards = false});

      //stop intake
      intake.stop();
   }
   }   
}

void Autonomous::auton5(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
   bool allianceStake = false;
   controller.print(0, 0, "skills");
   //autonomous 5 --> skills 
   // starts right against alliance stake
   //release first stage
   intake.autoRun(-1, 600);
   pros::delay(150);
   intake.stop();

   chassis.setPose(0, 0, 0);
   intake.autoRun(1, 600);
   chassis.moveToPose(-16.579, 11.954, 90, 5000, {.forwards = false});
   pros::delay(500);
   clamp.toggle();
   pros::delay(500);
   chassis.moveToPoint(-23.497, 34.635, 5000);
   chassis.moveToPoint(-60.122, 59.176, 5000);
   chassis.moveToPoint(-48.782, 35.937, 5000);
   chassis.moveToPoint(-48.596, 11.954, 5000);
   chassis.moveToPoint(-48.596, 0.055, 5000);
   chassis.moveToPoint(-60.122,11.582, 5000);
   chassis.moveToPoint(-59.565, 27.943, 5000);
   chassis.moveToPoint(-59.007, -0.13, 5000, {.forwards = false});

}


void Autonomous::autonMove(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
    switch (Autonomous::auton) {
   case blueRight:
      auton1(intake, clamp, doinker, lift);
      break;
   case redLeft:
      auton2(intake, clamp, doinker, lift);
      break;
   case redRight:
      auton3(intake, clamp, doinker, lift);
      break;
   case blueLeft:
      auton4(intake, clamp, doinker, lift);
      break;
   case skills:
      auton5(intake, clamp, doinker, lift);
      break;
   }
}

void Autonomous::autonSwitcher(int autonNum)
{
	switch (autonNum) {
	case 1:
		Autonomous::autonName = "Red Left";
		Autonomous::auton	  = redLeft;
      Autonomous::allianceColor = std::string("red");
		break;
	case 2:
		Autonomous::autonName = "Red Right";
		Autonomous::auton	  = redRight;
      Autonomous::allianceColor = std::string("red");
		break;
	case -1:
		Autonomous::autonName = "Blue Left";
		Autonomous::auton	  = blueLeft;
      Autonomous::allianceColor = std::string("blue");
		break;
	case -2:
		Autonomous::autonName = "Blue Right";
		Autonomous::auton	  = blueRight;
      Autonomous::allianceColor = std::string("blue");
		break;
   case 0:
      Autonomous::autonName = "Skills";
      Autonomous::auton    = skills;
      Autonomous::allianceColor = std::string("red");
	}
	std::cout << "Current auton: " + Autonomous::autonName << std::endl;
}