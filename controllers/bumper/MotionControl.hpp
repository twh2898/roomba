#pragma once

#include <webots/Motor.hpp>

#include "Localizer.hpp"

namespace roomba {
    using webots::Motor;

    class MotionControl {
        Motor * left;
        Motor * right;
        Localizer & loc;

    public:
        MotionControl(Motor * left, Motor * right, Localizer & loc)
            : left(left), right(right), loc(loc) {}
    };
}
