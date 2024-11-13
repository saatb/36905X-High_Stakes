#include "pros/misc.h"
#include "robot/optical.h"
#include <cassert>
#include "globals.h"
#include "pros/motors.h"
#include "pros/optical.hpp"

using namespace Robot;
using namespace Robot::Global;

Optical::Optical(){
}

bool Optical::isClose(){
    if (optical.get_proximity() > 180){
        return true;
    }
    else {
        return false;
    }
}