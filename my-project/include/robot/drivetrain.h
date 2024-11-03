#pragma once

namespace Robot{

    class Drivetrain {
        public:
        Drivetrain();
        enum DRIVE_MODE { CURVATURE_DRIVE = 0, ARCADE_DRIVE = 1, TANK_DRIVE = 2 };

	    static DRIVE_MODE driveMode;
        void run();

        private:
        void tankDrive();

    };
}