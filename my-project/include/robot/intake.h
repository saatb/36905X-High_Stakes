#pragma once

#include <string>
namespace Robot {

    class Intake  {
        public:
        Intake();
        void run(std::string allianceColor);
        void autoRun(int direction, int speed);
        void stop();
    };
}