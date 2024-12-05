#include "robot/doinker.h"

#include "globals.h"
#include "pros/misc.h"

using namespace Robot;
using namespace Robot::Global;

void Doinker::run(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
        Doinker::toggle();
    }
}

Doinker::Doinker(){}

void Doinker::toggle() {doinker.toggle();}