#pragma once

#include <cmath>

namespace robbie {
    using std::sqrt;

    struct XY {
        double x;
        double y;

        XY(double x = 0, double y = 0) : x(x), y(y) {}

        double length(const XY & other) {
            double dx = other.x - x;
            double dy = other.y - y;
            return sqrt(dx * dx + dy * dy);
        }
    };
}
