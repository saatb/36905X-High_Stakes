#pragma once

namespace Robot {

    class Intake  {
        public:
        Intake();
        void run();
        void autoRun(int direction, int intakeSpeed = 200, int conveyorSpeed = 600);
        void stop();
        void wallstake();
    };
}