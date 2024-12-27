#pragma once

namespace Robot {

    class Intake  {
        public:
        Intake();
        void run();
        void autoRun(int direction = 1, int intakeSpeed = 200, int conveyorSpeed = 600);
        void stop();
        void sort();
        void antiStall(int goal);
        void enableAutoSort();
        void disableAutoSort();
    };
}