#pragma once

#include <string>
namespace Robot {

    class Intake  {
        public:
        Intake();
        void run(std::string allianceColor = "red");
        void autoRun(int direction, int speed, std::string allianceColor = "red");
        void stop();
    };
}