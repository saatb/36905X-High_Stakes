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


Autonomous::routine Autonomous::auton = redElims;
std::string				  Autonomous::autonName;
std::string Autonomous::allianceColor = "red";

bool quals = false;
bool ringRush = true;
bool blueCorner = true;
bool redCenter = true;
/*
NOTES:
   - imu is mounted horizontally, so x and y are FLIPPED from path.jerryio
   - towards negative corner is NEGATIVE x, toward positive corner is POSITIVE x (opposite as path.jerryio)
   - towards blue alliance is POSITIVE y, towards red alliance is NEGATIVE y (same as path.jerryio)
   - RED to BLUE transformation is inverting y-values and 180 - original angle
*/

void Autonomous::auton1(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){ //blueright DONE - maybe elims mods
   //blueRight
   //score alliance stake
   chassis.setPose(0, 0, 180-140);
   lift.setPosition(2);
   chassis.moveToPoint(3, 6, 5000);
   lift.setPosition(3);

   //move to goal and clamp
   chassis.moveToPoint(-14, -30, 5000, {.forwards = false, .maxSpeed = 65});
   pros::delay(1150);
   clamp.toggle();
   lift.setPosition(0);
   intake.autoRun(1, 200, 600);

   //turn towards rings
   chassis.turnToHeading(180-320, 1000);
   
   //first ring
   chassis.moveToPoint(-30, -45, 2000, {.maxSpeed = 60});
   //second ring
   chassis.moveToPose(-48, -45.5, 180-270, 2000, {.maxSpeed = 60});
   pros::delay(1000);
   //move back
   chassis.moveToPoint(-30, -40, 2000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 4});
   chassis.moveToPoint(-20, -30, 2000, {.forwards = false});
   pros::delay(500);

   //third ring
   chassis.moveToPoint(-35, -30, 2000, {.maxSpeed = 60});

   pros::delay(2500);

   chassis.moveToPoint(-3, -38, 5000, {.forwards = false, .maxSpeed = 50});
   lift.setPosition(2);
   
   }

void Autonomous::auton2(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){ //redleft DONE - maybe elims mods
   //redLeft
   //score alliance stake
   chassis.setPose(0, 0, 140);
   lift.setPosition(2);
   chassis.moveToPoint(3, -6, 5000);
   lift.setPosition(3);
   pros::delay(400);

   //move to goal and clamp
   chassis.moveToPoint(-14, 30, 5000, {.forwards = false, .maxSpeed = 65});
   pros::delay(1150);
   clamp.toggle();
   lift.setPosition(0);
   intake.autoRun(1, 200, 600);

   //turn towards rings
   chassis.turnToHeading(320, 1000);
   
   //first ring
   chassis.moveToPoint(-32, 44, 2000, {.maxSpeed = 80, .minSpeed = 10, .earlyExitRange = 2});
   //second ring
   chassis.moveToPose(-52, 44, 270, 2000, {.maxSpeed = 80});
   pros::delay(1000);
   //move back
   chassis.moveToPoint(-30, 40, 2000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 4});
   chassis.moveToPoint(-20, 30, 2000, {.forwards = false, .maxSpeed = 80});
   pros::delay(500);

   //third ring
   chassis.moveToPoint(-37, 30, 2000, {.maxSpeed = 60});

   pros::delay(2500);

   chassis.moveToPoint(-3, 40, 5000, {.forwards = false, .maxSpeed = 50});
   lift.setPosition(2);
   /*
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
   chassis.moveToPose(-48, 45, 270, 2000, {.maxSpeed = 60});
   pros::delay(1000);
   chassis.moveToPoint(-20, 30, 2000, {.forwards = false});
   pros::delay(500);
   chassis.moveToPoint(-35, 30, 2000, {.maxSpeed = 60});

   pros::delay(2500);

   chassis.moveToPoint(-3, 42, 5000, {.forwards = false, .maxSpeed = 50});
   lift.setPosition(2);
   */

}

void Autonomous::auton3(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){ //redright DONE
   //redRight

   chassis.setPose(0, 0, 220);
   lift.setPosition(2);
   chassis.moveToPoint(-3, -6, 5000);
   lift.setPosition(3);

   //move to goal and clamp
   chassis.moveToPoint(14.3, 30, 5000, {.forwards = false, .maxSpeed = 65});
   pros::delay(1100);
   lift.setPosition(0);
   clamp.toggle();
   intake.autoRun(1, 200, 600);

   chassis.moveToPoint(40, 30, 5000, {.maxSpeed = 65});

   chassis.moveToPoint(38, 30, 1000, {.forwards = false});

   pros::delay(1000);
/*
   pros::delay(1000);

   //clear corner

   chassis.moveToPoint(38, 30, 5000, {.forwards = false});
*/  

   if (redCenter){ //grab center ring
         chassis.moveToPose(1, 47, 313, 2500);
         pros::delay(800);
         intakeMotor.move_velocity(0);
         doinker.toggle();
         //chassis.turnToHeading(290, 1000);
         chassis.moveToPoint(22, 27, 5000, {.forwards = false});
         //chassis.turnToHeading(350, 1000);
         pros::delay(1000);
         doinker.toggle();
         intake.autoRun();
         pros::delay(200);
         if (quals){
         chassis.moveToPoint(16, 46.5, 4000, {.maxSpeed = 65});
         }
         else if (!quals){
            //TODO: pick up ring, clamp mid goal and score
            conveyorMotor.move_velocity(0);
            clamp.toggle();
            chassis.moveToPoint(20, 38, 2000);
            chassis.moveToPose(40, 47, 180, 2000, {.forwards = false});
            pros::delay(2500);
            clamp.toggle();
            conveyorMotor.move_velocity(600);
            pros::delay(300);
            chassis.moveToPoint(40, 38, 2000);
         }

   }
   else { //corner clear
      chassis.moveToPoint(54, -6, 1700, {.maxSpeed = 65});

      doinker.toggle();   
      
      chassis.turnToHeading(40, 2000);
      
      pros::delay(1500);
      
      doinker.toggle();
      
      chassis.moveToPose(38, 18, 180, 2000, {.maxSpeed = 80});
   
      if (quals){ //touch ladder
      lift.setPosition(2);
      chassis.moveToPoint(3, 38, 5000, {.forwards = false, .maxSpeed = 50});}
   
      else if (!quals) {   //prep for third goal
      //start
      chassis.turnToHeading(0, 1000);

      pros::delay(2000);

      intake.stop();

      clamp.toggle();

      pros::delay(500);

      chassis.turnToHeading(180, 1000);

      chassis.moveToPoint(40, 36, 5000, {.forwards = false, .maxSpeed = 65});
      }
}

}

void Autonomous::auton4(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){ //blueleft DONE
   
   //blueLeft
   chassis.setPose(0, 0, 180-220);
   lift.setPosition(2);
   chassis.moveToPoint(-3, 6, 5000);
   lift.setPosition(3);

   pros::delay(800);

   //move to goal and clamp
   chassis.moveToPoint(14, -30, 5000, {.forwards = false, .maxSpeed = 65});
   pros::delay(1150);
   lift.setPosition(0);
   clamp.toggle();
   intake.autoRun(1, 200, 600);

   chassis.moveToPoint(40, -30, 5000, {.maxSpeed = 65});

   pros::delay(1000);

   chassis.moveToPoint(36, -30, 5000, {.forwards = false});

if (blueCorner){
   //clear corner

   chassis.moveToPoint(52, 4, 3000, {.maxSpeed = 65}); //y = 4, ms = 65
   
   pros::delay(800);

   intakeMotor.move_velocity(0);

   doinker.toggle();
   pros::delay(300);
   clamp.toggle();

   conveyorMotor.move_velocity(0);   

   chassis.turnToHeading(180-220, 2000);

   pros::delay(500);

   doinker.toggle();

   intake.autoRun(1, 200, 0);
   
   pros::delay(300);

   chassis.moveToPoint(48, 14, 2000);
   
   pros::delay(1000);
   if (quals){
      lift.setPosition(2);
      chassis.moveToPoint(5, -38, 5000, {.forwards = false, .maxSpeed = 50});
   }
   else if (!quals) {   
   //start
   chassis.moveToPose(28, -24, 180-180, 3000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2});

   chassis.moveToPose(38, -45, 180-180, 2500, {.forwards = false});
   
   pros::delay(2000);
   clamp.toggle();
   pros::delay(300);
   intake.autoRun();
   }
}
/* middle ring doesnt work :(
   chassis.turnToHeading(270, 500);

   chassis.moveToPose(7.5, -44, 200, 2200, {.maxSpeed = 80});

   doinker.toggle();

   intake.stop();

   chassis.turnToHeading(245, 500);

   chassis.turnToHeading(210, 500);

   chassis.moveToPose(7.5, -38, 210, 1000, {.forwards = false});

   chassis.moveToPose(9, -52, 210, 3000);
*/
}

void Autonomous::auton5(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){ //skills
   chassis.setPose(0, -1, 180);
   intake.disableAutoSort();

//score alliance stake
   lift.setPosition(1);
   intake.disableAntiStall();
   intake.autoRun();
   pros::delay(500);
   intake.stop();
   lift.setPosition(4);
   pros::delay(500);

//mogo
   chassis.moveToPoint(0, 6, 1000, {.forwards = false});
   chassis.turnToHeading(90, 500);
   chassis.moveToPoint(-25, 6, 2000, {.forwards = false, .maxSpeed = 55});
   lift.setPosition(0);
   pros::delay(950);
   clamp.toggle();
   intake.enableAntiStall();

//start scoring rings
   chassis.turnToHeading(0, 500);
   intake.autoRun();
   chassis.moveToPoint(-22, 30, 1500, {.minSpeed = 30, .earlyExitRange = 4}); //max speed 60, motion chain

//second ring
   chassis.turnToHeading(315, 500);
   chassis.moveToPoint(-36, 48, 2000, {.minSpeed = 70, .earlyExitRange = 4});
   chassis.moveToPoint(-44, 80, 2000);

//wall stake 1
   chassis.turnToHeading(180, 500, {.minSpeed = 20, .earlyExitRange = 5});
   lift.setPosition(1);
   intake.disableAntiStall();
   chassis.moveToPoint(-37, 56, 1000);
   chassis.turnToHeading(270, 500);
   chassis.moveToPoint(-64, 56, 2000);
   pros::delay(200);
   conveyorMotor.move_velocity(0);   
   pros::delay(400);
   conveyorMotor.move_relative(-50, -600);
   lift.setPosition(4);
   pros::delay(400);
   intake.autoRun();
   intake.enableAntiStall();

   //chassis.setPose(-64, 57, 270); //reset
   chassis.moveToPoint(-50, 56, 2000, {.forwards = false});
   pros::delay(500);
   lift.setPosition(0);
   chassis.turnToHeading(180, 500);

//3 rings
   chassis.moveToPoint(-48, 20, 2500, {.minSpeed = 65, .earlyExitRange = 4}); //motion chain
   chassis.moveToPoint(-48, 0, 1000, {.maxSpeed = 55});
   chassis.moveToPoint(-48, -9, 700, {.maxSpeed = 40, .minSpeed = 20, .earlyExitRange = 1});

   chassis.moveToPoint(-43, 5, 1000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 2}); //back up

//1 ring
   chassis.moveToPoint(-65, 5, 1000, {.minSpeed = 30, .earlyExitRange = 2}); //max speed 60

//cornering
   chassis.moveToPoint(-55, 5, 5000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2}); //max speed 50
   chassis.turnToHeading(45, 500);
   chassis.moveToPoint(-64, -5, 1000, {.forwards = false, .maxSpeed = 70});
   pros::delay(1000);
   clamp.toggle();
   intake.stop();
   conveyorMotor.move_velocity(-600);
   pros::delay(300);
   conveyorMotor.move_velocity(-600);

//leave corner
   chassis.moveToPoint(-10, 13, 5000, {.minSpeed = 70, .earlyExitRange = 4}); //motionchained


//mogo 2
   chassis.moveToPoint(21, 13, 3000, {.forwards = false, .minSpeed = 60}); //max speed 70
   pros::delay(950);
   clamp.toggle();
   pros::delay(300);
   intake.autoRun();
   
//first ring close to mogo
   chassis.moveToPoint(24, 33, 2000, {.minSpeed = 20, .earlyExitRange = 2}); //max speed 65

//ring under ladder
   chassis.moveToPoint(-3, 60, 2000); //-6, 55
   chassis.moveToPoint(18, 35, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3});

//ring to right of first ring
   //chassis.turnToHeading(90, 1000);
   chassis.moveToPoint(53, 43, 2000); //max speed 65
   pros::delay(1000);
   lift.setPosition(1);
   intake.disableAntiStall();


   chassis.moveToPoint(40, 40, 1000, {.forwards = false}); //max speed 65
   chassis.turnToHeading(0, 500);

   //wall stake 2
   chassis.moveToPoint(37, 65, 2000);
   chassis.turnToHeading(90, 500);
   chassis.moveToPoint(60, 65, 2000);
   //chassis.moveToPose(60, 55.5, 90, 3500);
   conveyorMotor.move_velocity(0);
   pros::delay(700);
   conveyorMotor.move_relative(-50, -600);
   lift.setPosition(4);
   pros::delay(500);
   intake.autoRun();
   chassis.moveToPoint(44, 65, 2000, {.forwards = false});
   //chassis.turnToHeading(90, 1000);
   chassis.turnToHeading(180, 500);
   lift.setPosition(0);
   intake.autoRun();
   intake.enableAntiStall();

   //2 rinfs
   chassis.moveToPoint(50, -13, 1900, {.maxSpeed = 70, .minSpeed = 20, .earlyExitRange = 4}); //max speed 65

   chassis.moveToPoint(40, 10, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 3});
   //chassis.turnToHeading(90, 1000);

   //1 ring
   chassis.moveToPoint(65, 16, 1000, {.minSpeed = 30, .earlyExitRange = 1}); //max speed 65

   chassis.moveToPoint(50, 12, 1500, {.forwards = false, .minSpeed = 20, .earlyExitRange = 4});

//cornering
   chassis.turnToHeading(315, 500);
   chassis.moveToPoint(70, -7, 1000, {.forwards = false});
   pros::delay(500);
   clamp.toggle();
   intake.stop();
   conveyorMotor.move_velocity(-600);
   pros::delay(300);

//leave corner, prep lady brown for alliance stake
   //chassis.moveToPoint(40, 6, 1000, {.minSpeed = 70, .earlyExitRange = 4});
   chassis.moveToPoint(46, 85, 3000);
   intake.autoRun();
   pros::delay(2000);
   lift.setPosition(1);
   intake.disableAntiStall();
   
//pick up second ring
   chassis.turnToHeading(270, 500);
   chassis.moveToPoint(20, 85, 1500);

//mogo 3
   chassis.turnToHeading(180, 500);
   chassis.moveToPoint(-4, 112, 2000, {.forwards = false});
   pros::delay(1000);
   clamp.toggle();
   //pros::delay(300);

//alliance stake + 1 ring on mogo
   chassis.turnToHeading(0, 500);
   chassis.moveToPose(-4, 130, 0, 1500, {.maxSpeed = 80});
   pros::delay(1000);
   chassis.setPose(-4, 130, 0);
   chassis.moveToPose(-4, 127, 0, 2000, {.forwards = false, .maxSpeed = 65});
   conveyorMotor.move_velocity(0);
   pros::delay(200);
   conveyorMotor.move_relative(-50, -600);
   pros::delay(400);
   intake.autoRun();
   lift.setPosition(4);
   intake.enableAntiStall();
   pros::delay(400);

   chassis.moveToPoint(-4, 120, 1500, {.forwards = false});
   chassis.turnToHeading(225, 500);
   lift.setPosition(0);

//solo ring on mogo
   chassis.moveToPoint(-28, 95, 2000, {.minSpeed = 20, .earlyExitRange = 4});

//first ring in 3
   chassis.moveToPoint(-52, 115, 2000);
   chassis.turnToHeading(270, 500);

//second ring in 3
   chassis.moveToPoint(-67, 117, 1000);
   chassis.moveToPoint(-56, 117, 1000, {.forwards = false});

//last ring in 3
//   chassis.moveToPoint(-53, 130, 2000);

//cornering
//   chassis.moveToPoint(-52, 120, 1000, {.forwards = false});
   chassis.moveToPoint(-65, 128, 1500);
   doinker.toggle();
   chassis.turnToHeading(220, 500);
   chassis.turnToHeading(120, 500);
   pros::delay(600);
   clamp.toggle();
   doinker.toggle();
   chassis.moveToPoint(-66, 130, 1000, {.forwards = false});
   intake.autoRun(-1);

//last mogo in corner

   chassis.moveToPoint(0, 110, 2000, {.minSpeed = 70, .earlyExitRange = 4});
   intake.autoRun();
   chassis.moveToPoint(70, 130, 4000);
   chassis.moveToPoint(62, 120, 1000, {.forwards = false});
}

void Autonomous::auton6(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){ //bluerightawp DONE
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
   intake.disableAntiStall();

   //move to mogo
   chassis.moveToPoint(32, -25, 5000, {.forwards = false, .maxSpeed = 65});
   conveyorMotor.move_velocity(0);
   pros::delay(1200);
   clamp.toggle();
   intake.enableAntiStall();
   
   pros::delay(200);
   intake.autoRun();

   chassis.moveToPoint(54, -26, 5000, {.maxSpeed = 60});

   pros::delay(1000);
   chassis.moveToPoint(27, -37, 5000, {.forwards = false});
}

void Autonomous::auton7(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){ //redleftawp DONE
   //autonomous 7 --> red left AWP
   //alliance stake, two on each mogo
   //lines up at 320 degrees on imu, as close as possible to rings in front of alliance stake

   //score alliance stake
   chassis.setPose(0, 0, 140);
   lift.setPosition(2);
   chassis.moveToPoint(3, -6, 5000);
   lift.setPosition(3);
   pros::delay(400);

   //move to goal and clamp
   chassis.moveToPoint(-14, 30, 5000, {.forwards = false, .maxSpeed = 65});
   pros::delay(1050);
   clamp.toggle();
   lift.setPosition(0);
   intake.autoRun(1, 200, 600);
   
   //chassis.turnToHeading(320, 1000);
   chassis.moveToPoint(-30, 44, 2000, {.maxSpeed = 60}); //-30, 44
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
   intake.disableAntiStall();
   conveyorMotor.move_velocity(0);

   //move to mogo
   chassis.moveToPoint(30, 24, 5000, {.forwards = false, .maxSpeed = 65});
   conveyorMotor.move_velocity(0);
   pros::delay(1230);
   clamp.toggle();
   
   pros::delay(200);
   intake.enableAntiStall();
   intake.autoRun();

   chassis.moveToPoint(54, 27, 5000, {.maxSpeed = 65});

   pros::delay(1200);
   chassis.moveToPoint(22, 39, 5000, {.forwards = false});
}

void Autonomous::auton8(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){ //redelims
   //autonomous 8 --> red elims rushes

   if (ringRush){
      //ring rush autonomous RED
      chassis.setPose(0, 0, 0);
      chassis.moveToPose(-.5, 29, 0, 4000, {.minSpeed = 80, .earlyExitRange = 2});
      intakeMotor.move_velocity(600);
      doinker.toggle();
      chassis.moveToPose(-13.5, 45, 332, 3000, {.minSpeed = 80});
      chassis.moveToPose(-21, 44, 270, 1500);

      pros::delay(400);
      chassis.moveToPoint(10.5, 30, 2000, {.forwards = false, .maxSpeed = 65});
      pros::delay(1150);
      clamp.toggle();
      pros::delay(300);
      chassis.turnToHeading(260, 500);
      pros::delay(500);
      doinker.toggle();
      intake.autoRun();

      chassis.turnToHeading(270, 500);
      chassis.moveToPoint(-22, 30, 2000, {.maxSpeed = 70});

      pros::delay(600);
      chassis.turnToHeading(135, 500);

      chassis.moveToPoint(9, 0, 2000);
      chassis.turnToHeading(90, 500);
      chassis.moveToPose(50, 0, 90, 4000, {.maxSpeed = 65});

   }
   else {
      //goal rush autonomous RED
   }

}

void Autonomous::auton9(Intake &intake, Clamp &clamp, Doinker &doinker, Lift &lift){ //blueelims
   //autonomous 9 --> blue elims rushes
   if (ringRush){
      //ring rush autonomous BLUE
   }
   else {
      //goal rush autonomous BLUE
   }

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
   case redElims:
      auton8(intake, clamp, doinker, lift);
      Autonomous::allianceColor = std::string("red");
      break;
   case blueElims:
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
		Autonomous::autonName = "Red Elims";
		Autonomous::auton	  = redElims;
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
		Autonomous::autonName = "Blue Elims";
		Autonomous::auton	  = blueElims;
      Autonomous::allianceColor = std::string("blue");
		break;
   
   case 0:
      Autonomous::autonName = "Skills";
      Autonomous::auton    = skills;
      Autonomous::allianceColor = std::string("red");
	}
	std::cout << "Current auton: " + Autonomous::autonName << std::endl;
}