#pragma once

namespace Robot {

    class Lift  {
        public:
        #define armGearRatio 4
        Lift();
        void init();
        void run();
        void setPosition(int newIndex);
        //void autoRun(int position);

        private:
        
    };
}