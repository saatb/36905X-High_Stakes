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


Autonomous::routine Autonomous::auton = skills;
std::string				  Autonomous::autonName;
std::string Autonomous::allianceColor = "red";

bool quals = true;
bool goalRush = false;

/*
NOTES:
   - imu is mounted horizontally, so x and y are FLIPPED from path.jerryio
   - towards negative corner is NEGATIVE x, toward positive corner is POSITIVE x (opposite as path.jerryio)
   - towards blue alliance is POSITIVE y, towards red alliance is NEGATIVE y (same as path.jerryio)
   - RED to BLUE transformation is inverting y-values (might also need to change headings)
*/

void Autonomous::auton1(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
   //blueRight
   //score alliance stake
   chassis.setPose(0, 0, 180 - 140);
   lift.setPosition(2);
   chassis.moveToPoint(3, 6, 5000);
   lift.setPosition(3);

   //move to goal and clamp
   chassis.moveToPoint(-14, -30, 5000, {.forwards = false, .maxSpeed = 65});
   lift.setPosition(0);
   pros::delay(1050);
   clamp.toggle();
   intake.autoRun(1, 200, 600);
   
   //pick up ring off of center line
   //chassis.turnToHeading(330, 1000);
   chassis.moveToPoint(-29.5, -44, 5000, {.maxSpeed = 50});
   pros::delay(3000);

   //back up slightly
   chassis.moveToPoint(-24, -34, 1000, {.forwards = false});

   //move to close ring stack
   chassis.moveToPoint(-42, -25.5, 5000, {.maxSpeed = 50});
   pros::delay(2400);

   //touch ladder
   chassis.moveToPoint(-7, -39, 5000, {.forwards = false, .maxSpeed = 50});
   lift.setPosition(2);

   }

void Autonomous::auton2(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
   //redLeft
   //score alliance stake
   chassis.setPose(0, 0, 140);
   lift.setPosition(2);
   chassis.moveToPoint(3, -6, 5000);
   lift.setPosition(3);

   //move to goal and clamp
   chassis.moveToPoint(-14, 30, 5000, {.forwards = false, .maxSpeed = 65});
   lift.setPosition(0);
   pros::delay(1050);
   clamp.toggle();
   intake.autoRun(1, 200, 600);
   
   chassis.moveToPoint(-29.5, 45, 5000, {.maxSpeed = 50});
   pros::delay(3000);

   //back up slightly
   chassis.moveToPoint(-22, 34, 1000, {.forwards = false});

   //move to close ring stack
   chassis.moveToPoint(-38, 23, 5000, {.maxSpeed = 50});
   pros::delay(2400);

   //touch ladder
   chassis.moveToPoint(-3, 39, 5000, {.forwards = false, .maxSpeed = 50});
   lift.setPosition(2);

}

void Autonomous::auton3(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
   //redRight

   chassis.setPose(0, 0, 220);
   lift.setPosition(2);
   chassis.moveToPoint(-3, -6, 5000);
   lift.setPosition(3);

   //move to goal and clamp
   chassis.moveToPoint(14, 30, 5000, {.forwards = false, .maxSpeed = 65});
   lift.setPosition(0);
   pros::delay(1050);
   clamp.toggle();
   intake.autoRun(1, 200, 600);

   chassis.moveToPoint(42, 30, 5000);

   pros::delay(3000);

   if (quals){
      lift.setPosition(2);
      chassis.moveToPoint(3, 38, 5000, {.forwards = false, .maxSpeed = 50});
      pros::delay(4000);
      intake.stop();
   }
   else if (!quals) {   
   chassis.turnToHeading(0, 1000);
   
   chassis.moveToPoint(42, 24, 5000, {.forwards = false, .maxSpeed = 65});

   pros::delay(1000);
   intake.stop();

   clamp.toggle();

   pros::delay(500);

   chassis.turnToHeading(180, 1000);

   chassis.moveToPoint(42, 38, 5000, {.forwards = false, .maxSpeed = 65});

   }

}

void Autonomous::auton4(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
   //blueLeft

   chassis.setPose(0, 0, 180-220);
   lift.setPosition(2);
   chassis.moveToPoint(-3, 6, 5000);
   lift.setPosition(3);

   //move to goal and clamp
   chassis.moveToPoint(14, -30, 5000, {.forwards = false, .maxSpeed = 65});
   lift.setPosition(0);
   pros::delay(1050);
   clamp.toggle();
   intake.autoRun(1, 200, 600);

   chassis.moveToPoint(42, -30, 5000);

   pros::delay(5000);

   if (quals){
      lift.setPosition(2);
      chassis.moveToPoint(3, -38, 5000, {.forwards = false, .maxSpeed = 50});
      pros::delay(4000);
      intake.stop();
   }
   else if (!quals) {   
   chassis.turnToHeading(180-0, 1000);
   
   chassis.moveToPoint(42, -24, 5000, {.forwards = false, .maxSpeed = 65});

   pros::delay(1000);

   intake.stop();

   clamp.toggle();

   pros::delay(500);

   chassis.turnToHeading(180-180, 1000);

   chassis.moveToPoint(42, -38, 5000, {.forwards = false, .maxSpeed = 65});

   }

}

void Autonomous::auton5(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
   chassis.setPose(0, 0, 180);
   //score alliance stake
   lift.setPosition(1);
   intake.disableAntiStall();
   intake.autoRun();
   pros::delay(500);
   intake.stop();
   lift.setPosition(4);
   pros::delay(1000);

   //mogo
   chassis.moveToPoint(0, 4, 1000, {.forwards = false});
   lift.setPosition(0);
   chassis.turnToHeading(90, 1000);
   chassis.moveToPoint(-24, 5, 5000, {.forwards = false, .maxSpeed = 55});
   pros::delay(950);
   clamp.toggle();
   intake.enableAntiStall();

   //start scoring rings
   chassis.turnToHeading(0, 1000);
   intake.autoRun();
   chassis.moveToPoint(-20, 28, 5000, {.maxSpeed = 60});

   chassis.turnToHeading(315, 1000);
   chassis.moveToPose(-36, 48, 0, 5000, {.minSpeed = 40, .earlyExitRange = 4});
   chassis.moveToPoint(-42, 78, 5000, {.maxSpeed = 45});
   pros::delay(2000);
   chassis.turnToHeading(220, 1000);
   chassis.moveToPose(-64, 60, 270, 5000, {.maxSpeed = 40});
   pros::delay(2000);
   lift.setPosition(1);
   intake.disableAntiStall();
   pros::delay(1500);
   intake.stop();
   lift.setPosition(4);
   pros::delay(1000);
   lift.setPosition(0);

   chassis.setPose(-64, 60, 270); //reset
   intake.enableAntiStall();
   intake.autoRun();
   chassis.moveToPose(-50, 59.5, 270, 2000, {.forwards = false});
   chassis.turnToHeading(180, 1000);
   chassis.moveToPoint(-52, -9, 5000, {.maxSpeed = 45});
   pros::delay(1000);

   chassis.moveToPoint(-42, 5, 5000, {.forwards = false, .maxSpeed = 55});
   chassis.turnToHeading(270, 1000);
   chassis.moveToPoint(-67, 5, 5000, {.maxSpeed = 60});
   pros::delay(1000);

   chassis.moveToPoint(-42, 5, 5000, {.forwards = false, .maxSpeed = 55});
   chassis.turnToHeading(45, 1000);
   chassis.moveToPoint(-64, -5, 5000, {.forwards = false, .maxSpeed = 70});
   pros::delay(1000);
   clamp.toggle();
   pros::delay(300);
   chassis.moveToPoint(-42, 5, 5000, {.maxSpeed = 70});
   chassis.turnToHeading(270, 1000);


   chassis.moveToPose(27, 5.7, 270, 3000, {.forwards = false, .maxSpeed = 70});
   pros::delay(2000);
   clamp.toggle();
   pros::delay(300);
   chassis.turnToHeading(0, 1000);
   chassis.moveToPoint(18, 28, 5000, {.maxSpeed = 65});
   pros::delay(500);
   chassis.turnToHeading(90, 1000);
   chassis.moveToPoint(50, 28, 5000, {.maxSpeed = 65});
   
   pros::delay(500);

   chassis.moveToPoint(40, 25, 5000, {.forwards = false, .maxSpeed = 65});
   chassis.turnToHeading(0, 1000);
   chassis.moveToPoint(56, 50, 5000, {.maxSpeed = 65});
   chassis.turnToHeading(90, 1000);
   chassis.turnToHeading(180, 1000);
   chassis.moveToPoint(44, -9, 5000, {.maxSpeed = 65});

   chassis.moveToPoint(40, 6, 3000, {.forwards = false});
   chassis.turnToHeading(90, 1000);
   chassis.moveToPoint(60, 6, 3000, {.maxSpeed = 65});
   pros::delay(1000);

   chassis.moveToPoint(40, 6, 3000, {.forwards = false});

   chassis.turnToHeading(315, 1000);
   chassis.moveToPoint(60, -15, 5000, {.forwards = false});
   pros::delay(500);
   clamp.toggle();
   pros::delay(300);
   chassis.moveToPoint(40, 6, 3000);
}

void Autonomous::auton6(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
   //DONE
   //autonomous 2 --> blueRight AWP (double mogo)
   //alliance stake, two on each mogo
   //lines up at 320 degrees on imu, as close as possible to rings in front of alliance stake
   //180 - theta, inv y

   //score alliance stake
   chassis.setPose(0, 0, 180 - 140);
   lift.setPosition(2);
   chassis.moveToPoint(3, 6, 5000);
   lift.setPosition(3);

   //move to goal and clamp
   chassis.moveToPoint(-14, -30, 5000, {.forwards = false, .maxSpeed = 65});
   lift.setPosition(0);
   pros::delay(1050);
   clamp.toggle();
   intake.autoRun(1, 200, 600);
   
   //pick up ring off of center line
   //chassis.turnToHeading(330, 1000);
   chassis.moveToPoint(-28.5, -43.5, 5000, {.maxSpeed = 60});
   pros::delay(1800);

   //back up slightly
   chassis.moveToPoint(-24, -34, 1000, {.forwards = false});

   //move to close ring stack
   chassis.moveToPoint(-38, -25.5, 5000);
   pros::delay(200);
   
   //move to alliance stake ring, motion chained
   chassis.moveToPoint(-4, -7, 5000, {.minSpeed = 55, .earlyExitRange = 4});
   chassis.moveToPoint(20, -7, 5000, {.maxSpeed = 45});
   double now = pros::millis();
   clamp.toggle();
   while (optical.get_hue() < 100 && (pros::millis() - now) < 2000) {
      intake.autoRun(1, 600, 400); 
   }
   intake.stop();

   //move to mogo
   chassis.moveToPoint(28, -23, 5000, {.forwards = false, .maxSpeed = 65});
   pros::delay(1200);
   clamp.toggle();
   
   pros::delay(200);
   intake.autoRun();

   chassis.moveToPoint(50, -26, 5000, {.minSpeed = 40});
   lift.setPosition(2);

   pros::delay(1000);
   chassis.moveToPoint(24, -39, 5000, {.forwards = false});
}

void Autonomous::auton7(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
   //autonomous 7 --> red left AWP
   //alliance stake, two on each mogo
   //lines up at 320 degrees on imu, as close as possible to rings in front of alliance stake

   //score alliance stake
   chassis.setPose(0, 0, 140);
   lift.setPosition(2);
   chassis.moveToPoint(3, -6, 5000);
   lift.setPosition(3);

   //move to goal and clamp
   chassis.moveToPoint(-14, 30, 5000, {.forwards = false, .maxSpeed = 65});
   lift.setPosition(0);
   pros::delay(1050);
   clamp.toggle();
   intake.autoRun(1, 200, 600);
   
   chassis.moveToPoint(-28.5, 44.5, 5000, {.maxSpeed = 60});
   pros::delay(1800);

   //back up slightly
   chassis.moveToPoint(-22, 34, 1000, {.forwards = false});

   //move to close ring stack
   chassis.moveToPoint(-38, 25.5, 5000);
   pros::delay(200);
   
   //move to alliance stake ring, motion chained
   chassis.moveToPoint(-4, 6, 5000, {.minSpeed = 63, .earlyExitRange = 4});
   chassis.moveToPoint(20, 6, 5000, {.maxSpeed = 45});
   double now = pros::millis();
   clamp.toggle();
   while (optical.get_hue() > 25 && (pros::millis() - now) < 2000) {
      intake.autoRun(1, 600, 400); 
   }
   intake.stop();

   //move to mogo
   chassis.moveToPoint(27, 22, 5000, {.forwards = false, .maxSpeed = 65});
   pros::delay(1200);
   clamp.toggle();
   
   pros::delay(200);
   intake.autoRun();

   chassis.moveToPoint(49, 27, 5000, {.minSpeed = 40});
   lift.setPosition(2);

   pros::delay(1200);
   chassis.moveToPoint(19, 38, 5000, {.forwards = false});
}

void Autonomous::auton8(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
   //autonomous 8 --> red right goal
   
   chassis.setPose(0, 0, 345);

   chassis.moveToPose(-16, 42, 330, 5000);
   intakeMotor.move_velocity(200);
   pros::delay(800);
   doinker.toggle();
   chassis.moveToPose(-16, 20, 0, 5000, {.forwards = false});
   pros::delay(800);
   doinker.toggle();

}

void Autonomous::auton9(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
   //autonomous 9 --> blue left goal

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
   case blueRightAWP:
      auton6(intake, clamp, doinker, lift);
      break;
   case redLeftAWP:
      auton7(intake, clamp, doinker, lift);
      break;
   case redRightGoal:
      auton8(intake, clamp, doinker, lift);
      break;
   case blueLeftGoal:
      auton9(intake, clamp, doinker, lift);
      break;
   case skills:
      auton5(intake, clamp, doinker, lift);
      break;
   }
}

void Autonomous::autonSwitcher(int autonNum){
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
   case 3:
		Autonomous::autonName = "Red Left AWP";
		Autonomous::auton	  = redLeftAWP;
      Autonomous::allianceColor = std::string("red");
   case 4:
		Autonomous::autonName = "Red Right Goal";
		Autonomous::auton	  = redRightGoal;
      Autonomous::allianceColor = std::string("red");
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
   case -3:
		Autonomous::autonName = "Blue Right AWP";
		Autonomous::auton	  = blueRightAWP;
      Autonomous::allianceColor = std::string("blue");
		break;
   case -4:
		Autonomous::autonName = "Blue Left Goal";
		Autonomous::auton	  = blueLeftGoal;
      Autonomous::allianceColor = std::string("blue");
		break;
   case 0:
      Autonomous::autonName = "Skills";
      Autonomous::auton    = skills;
      Autonomous::allianceColor = std::string("red");
	}
	std::cout << "Current auton: " + Autonomous::autonName << std::endl;
}