#pragma once

#include <iostream>
#include <webots/Accelerometer.hpp>
#include <webots/Motor.hpp>

#include "Roomba.hpp"

namespace roomba {
    using webots::Accelerometer;
    using webots::Motor;

    class Localizer {
    public:
        double velX;
        double velY;
        double posX;
        double posY;

    public:
        Localizer() : velX(0), velY(0), posX(0), posY(0) {}

        void update(Roomba * roomba) {
            auto t = roomba->gyro->getSamplingPeriod();
            auto * values = roomba->gyro->getValues();
            double dx = values[0] * t;
            double dy = values[1] * t;

            velX += dx;
            velY += dy;

            if (roomba->leftMotor->getVelocity() == 0
                && roomba->rightMotor->getVelocity() == 0) {
                std::cout << "Velocity 0" << std::endl;
                velX = 0;
                velY = 0;
            }
        }
    };
}
