#pragma once

#include <iostream>
#include <webots/Accelerometer.hpp>
#include <webots/Motor.hpp>

namespace roomba {
    using webots::Accelerometer;
    using webots::Motor;

    class Localizer {
    public:
        Motor * left;
        Motor * right;
        Accelerometer * accel;
        double velX;
        double velY;
        double posX;
        double posY;

    public:
        Localizer(Accelerometer * accel, Motor * left, Motor * right)
            : accel(accel),
              left(left),
              right(right),
              velX(0),
              velY(0),
              posX(0),
              posY(0) {}

        void update() {
            auto t = accel->getSamplingPeriod();
            auto * values = accel->getValues();
            double dx = values[0] * t;
            double dy = values[1] * t;

            velX += dx;
            velY += dy;

            if (left->getVelocity() == 0 && right->getVelocity() == 0) {
                std::cout << "Velocity 0" << std::endl;
                velX = 0;
                velY = 0;
            }
        }
    };
}
