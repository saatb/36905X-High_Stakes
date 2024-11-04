#pragma once

namespace Robot {

    class Intake  {
        public:
        Intake ();
        void run();
        void autoRun(int direction, int speed);
        void stop();
    };
}