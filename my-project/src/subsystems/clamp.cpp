#include "robot/clamp.h"

#include "globals.h"
#include "pros/misc.h"

using namespace Robot;
using namespace Robot::Global;

void Clamp::run(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
        Clamp::toggle();
    }
}

Clamp::Clamp(){}

void Clamp::toggle() {clampControl.toggle();}