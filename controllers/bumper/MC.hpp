#pragma once

#include <webots/Motor.hpp>

#include "Loc.hpp"

namespace roomba {
    using webots::Motor;

    class MC {
        Motor * left;
        Motor * right;
        Localizer & loc;

    public:
        MC(Motor * left, Motor * right, Localizer & loc)
            : left(left), right(right), loc(loc) {}
    };
}
