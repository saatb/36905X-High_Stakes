#pragma once

namespace Robot{

    class Drivetrain {
        public:
        Drivetrain();

        void run();

        private:
        void tankDrive();

    };
}