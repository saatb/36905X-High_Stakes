#pragma once
#include <string>
namespace Robot{
    class Optical {
        public:
        Optical();
        enum colors {red, blue};
        bool isClose();
        bool isOppositeColor(std::pmr::string allianceColor);
    };
}