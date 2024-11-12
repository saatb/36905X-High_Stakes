#pragma once
#include <string>
namespace Robot{
    class Optical {
        public:
        Optical();
        std::string checkDistance();
        std::string checkColor(std::pmr::string allianceColor);
    };
}