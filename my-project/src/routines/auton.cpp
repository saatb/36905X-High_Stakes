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


Autonomous::routine Autonomous::auton = blueRightAWP;
std::string				  Autonomous::autonName;
std::string Autonomous::allianceColor = "blue";

bool quals = true;

/*
NOTES:
   - imu is mounted horizontally, so x and y are FLIPPED from path.jerryio
   - towards negative corner is NEGATIVE x, toward positive corner is POSITIVE x (opposite as path.jerryio)
   - towards blue alliance is POSITIVE y, towards red alliance is NEGATIVE y (same as path.jerryio)
   - RED to BLUE transformation is inverting y-values and 180 - original angle
*/

void Autonomous::auton1(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
   //blueRight
   //score alliance stake
   chassis.setPose(0, 0, 180-140);
   lift.setPosition(2);
   chassis.moveToPoint(3, 6, 5000);
   lift.setPosition(3);

   //move to goal and clamp
   chassis.moveToPoint(-14, -30, 5000, {.forwards = false, .maxSpeed = 65});
   pros::delay(1050);
   clamp.toggle();
   lift.setPosition(0);
   intake.autoRun(1, 200, 600);

   chassis.turnToHeading(180-320, 1000);
   
   chassis.moveToPoint(-30, -45, 2000, {.maxSpeed = 60});
   chassis.moveToPose(-48, -46.5, 180-270, 2000, {.maxSpeed = 60});
   pros::delay(1000);
   chassis.moveToPoint(-20, -30, 2000, {.forwards = false});
   pros::delay(500);
   chassis.moveToPoint(-35, -30, 2000, {.maxSpeed = 60});

   pros::delay(2500);

   chassis.moveToPoint(-3, -42, 5000, {.forwards = false, .maxSpeed = 50});
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
   pros::delay(1050);
   clamp.toggle();
   lift.setPosition(0);
   intake.autoRun(1, 200, 600);

   chassis.turnToHeading(320, 1000);
   
   chassis.moveToPoint(-30, 45, 2000, {.maxSpeed = 60});
   chassis.moveToPose(-48, 46.5, 270, 2000, {.maxSpeed = 60});
   pros::delay(1000);
   chassis.moveToPoint(-20, 30, 2000, {.forwards = false});
   pros::delay(500);
   chassis.moveToPoint(-35, 30, 2000, {.maxSpeed = 60});

   pros::delay(2500);

   chassis.moveToPoint(-3, 42, 5000, {.forwards = false, .maxSpeed = 50});
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
   pros::delay(1050);
   lift.setPosition(0);
   clamp.toggle();
   intake.autoRun(1, 200, 600);

   chassis.moveToPoint(42, 30, 5000, {.maxSpeed = 65});

   pros::delay(1000);

   //clear corner

   chassis.moveToPoint(38, 30, 5000, {.forwards = false});

   chassis.moveToPoint(52, -4, 5000, {.maxSpeed = 65});

   doinker.toggle();   

   chassis.turnToHeading(40, 2000);

   chassis.moveToPose(38, 18, 180, 2000, {.forwards = false, .maxSpeed = 65});

   doinker.toggle();

   if (quals){
      lift.setPosition(2);
      chassis.moveToPoint(3, 38, 5000, {.forwards = false, .maxSpeed = 50});
   }
   else if (!quals) {   
   //start

   pros::delay(2000);

   intake.stop();

   clamp.toggle();

   pros::delay(500);

   chassis.turnToHeading(180, 1000);

   chassis.moveToPoint(40, 36, 5000, {.forwards = false, .maxSpeed = 65});
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
   pros::delay(1050);
   lift.setPosition(0);
   clamp.toggle();
   intake.autoRun(1, 200, 600);

   chassis.moveToPoint(42, -30, 5000, {.maxSpeed = 65});

   pros::delay(1000);

   //clear corner

   chassis.moveToPoint(38, -30, 5000, {.forwards = false});

   chassis.moveToPoint(52, 4, 5000, {.maxSpeed = 65});
   
   pros::delay(1000);

   intakeMotor.move_velocity(0);

   doinker.toggle();

   clamp.toggle();

   conveyorMotor.move_velocity(0);   

   chassis.turnToHeading(180-220, 2000);

   pros::delay(1000);

   doinker.toggle();

   intake.autoRun(1, 200, 0);

   chassis.moveToPoint(48, 8, 2000);

   pros::delay(1000);
   if (quals){
      lift.setPosition(2);
      chassis.moveToPoint(3, -38, 5000, {.forwards = false, .maxSpeed = 50});
   }
   else if (!quals) {   
   //start
   chassis.moveToPose(30, -18, 180-180, 2000, {.forwards = false, .maxSpeed = 65});

   pros::delay(2000);

   chassis.turnToHeading(180-180, 1000);

   chassis.moveToPoint(32, -36, 5000, {.forwards = false, .maxSpeed = 65});
   }

}

void Autonomous::auton5(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
   chassis.setPose(0, -1, 180);
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
   chassis.moveToPoint(-44, 78, 5000, {.maxSpeed = 45});
   pros::delay(2000);
   chassis.turnToHeading(220, 1000);
   chassis.moveToPose(-64, 58.5, 270, 5000, {.maxSpeed = 50});
   pros::delay(2000);
   lift.setPosition(1);
   intake.disableAntiStall();
   pros::delay(1500);
   intake.stop();
   lift.setPosition(4);
   pros::delay(1000);
   lift.setPosition(0);

   chassis.setPose(-64, 58.5, 270); //reset
   intake.enableAntiStall();
   intake.autoRun();
   chassis.moveToPose(-50, 58.5, 270, 2000, {.forwards = false});
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
   intake.stop();
   conveyorMotor.move_relative(-100, -600);
   pros::delay(300);
   chassis.moveToPoint(-42, 5, 5000, {.maxSpeed = 70});
   chassis.turnToHeading(270, 1000);


   chassis.moveToPose(27, 5.7, 270, 3000, {.forwards = false, .maxSpeed = 70});
   pros::delay(2000);
   clamp.toggle();
   pros::delay(300);
   chassis.turnToHeading(0, 1000);
   intake.autoRun();
   chassis.moveToPoint(18, 28, 5000, {.maxSpeed = 65});
   pros::delay(500);
   chassis.turnToHeading(90, 1000);
   chassis.moveToPoint(50, 31, 5000, {.maxSpeed = 65});
   
   pros::delay(500);

   chassis.moveToPoint(40, 25, 5000, {.forwards = false, .maxSpeed = 65});
   chassis.turnToHeading(0, 1000);
   chassis.moveToPoint(52, 60, 5000, {.maxSpeed = 65});
   chassis.turnToHeading(90, 1000);
   chassis.turnToHeading(180, 1000);
   chassis.moveToPoint(47, -9, 5000, {.maxSpeed = 65});

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
   pros::delay(1200);
   clamp.toggle();
   intake.autoRun(1, 200, 600);
   lift.setPosition(0);
   
   //pick up ring off of center line
   //chassis.turnToHeading(330, 1000);
   chassis.moveToPoint(-28.5, -43.5, 5000, {.maxSpeed = 60});
   pros::delay(1800);

   //back up slightly
   chassis.moveToPoint(-23, -34, 1000, {.forwards = false});

   //move to close ring stack
   chassis.moveToPoint(-39.2, -26.5, 5000, {.maxSpeed = 60});
   pros::delay(400);
   chassis.moveToPoint(-37, -26.5, 5000, {.forwards = false});
   pros::delay(1000);
   
   //move to alliance stake ring, motion chained
   chassis.moveToPoint(-4, -8, 5000, {.minSpeed = 55, .earlyExitRange = 4});
   chassis.moveToPoint(20, -6, 5000, {.maxSpeed = 45});
   double now = pros::millis();
   clamp.toggle();
   lift.setPosition(2);
   while (optical.get_hue() < 100 && (pros::millis() - now) < 2000) {
      intake.autoRun(1, 600, 400); 
   }
   conveyorMotor.move_velocity(0);

   //move to mogo
   chassis.moveToPoint(32, -25, 5000, {.forwards = false, .maxSpeed = 65});
   pros::delay(1200);
   clamp.toggle();
   
   pros::delay(200);
   intake.autoRun();

   chassis.moveToPoint(54, -26, 5000, {.maxSpeed = 60});

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
   pros::delay(1050);
   clamp.toggle();
   lift.setPosition(0);
   intake.autoRun(1, 200, 600);
   
   chassis.turnToHeading(320, 1000);
   chassis.moveToPoint(-30, 45, 2000, {.maxSpeed = 60});
   pros::delay(1200);

   //back up slightly
   chassis.moveToPoint(-22, 34, 1000, {.forwards = false});

   //move to close ring stack
   chassis.moveToPoint(-38, 27, 5000);
   pros::delay(1200);
   
   //move to alliance stake ring, motion chained
   chassis.moveToPoint(-4, 6, 5000, {.minSpeed = 63, .earlyExitRange = 4});
   chassis.moveToPoint(20, 6, 5000, {.maxSpeed = 45});
   double now = pros::millis();
   clamp.toggle();
   lift.setPosition(2);
   while (optical.get_hue() > 25 && (pros::millis() - now) < 2000) {
      intake.autoRun(1, 600, 400); 
   }
   conveyorMotor.move_velocity(0);

   //move to mogo
   chassis.moveToPoint(27, 22, 5000, {.forwards = false, .maxSpeed = 65});
   pros::delay(1300);
   clamp.toggle();
   
   pros::delay(200);
   intake.autoRun();

   chassis.moveToPoint(53, 27, 5000, {.maxSpeed = 65});

   pros::delay(1200);
   chassis.moveToPoint(21, 42, 5000, {.forwards = false});
}

void Autonomous::auton8(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
   //autonomous 8 --> red right goal
   
   chassis.setPose(0, 0, 180);
   chassis.moveToPoint(0, 10, 5000, {.forwards = false});

}

void Autonomous::auton9(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
   //autonomous 9 --> blue leave
   
   chassis.setPose(0, 0, 180);
   pros::delay(5000);
   chassis.moveToPoint(0, 10, 5000, {.forwards = false});

}

void Autonomous::autonMove(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){
    switch (Autonomous::auton) {
   case blueRight:
      auton1(intake, clamp, doinker, lift);
      Autonomous::allianceColor = std::string("blue");
      break;
   case redLeft:
      auton2(intake, clamp, doinker, lift);
      Autonomous::allianceColor = std::string("red");
      break;
   case redRight:
      auton3(intake, clamp, doinker, lift);
      Autonomous::allianceColor = std::string("red");
      break;
   case blueLeft:
      auton4(intake, clamp, doinker, lift);
      Autonomous::allianceColor = std::string("blue");
      break;
   case blueRightAWP:
      auton6(intake, clamp, doinker, lift);
      Autonomous::allianceColor = std::string("blue");
      break;
   case redLeftAWP:
      auton7(intake, clamp, doinker, lift);
      Autonomous::allianceColor = std::string("red");
      break;
   case redLeave:
      auton8(intake, clamp, doinker, lift);
      Autonomous::allianceColor = std::string("red");
      break;
   case blueLeave:
      auton9(intake, clamp, doinker, lift);
      Autonomous::allianceColor = std::string("blue");
      break;
   case skills:
      auton5(intake, clamp, doinker, lift);
      Autonomous::allianceColor = std::string("red");
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
      break;
   
   case 4:
		Autonomous::autonName = "Red Leave";
		Autonomous::auton	  = redLeave;
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
   case -3:
		Autonomous::autonName = "Blue Right AWP";
		Autonomous::auton	  = blueRightAWP;
      Autonomous::allianceColor = std::string("blue");
		break;
   
   case -4:
		Autonomous::autonName = "Blue Leave";
		Autonomous::auton	  = blueLeave;
      Autonomous::allianceColor = std::string("blue");
		break;
   
   case 0:
      Autonomous::autonName = "Skills";
      Autonomous::auton    = skills;
      Autonomous::allianceColor = std::string("red");
	}
	std::cout << "Current auton: " + Autonomous::autonName << std::endl;
}