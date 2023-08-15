#pragma once

#include <iostream>
#include <webots/Accelerometer.hpp>
#include <webots/Motor.hpp>

#include "Roomba.hpp"
#include "Telemetry.hpp"

namespace roomba {
    using webots::Accelerometer;
    using webots::Motor;

    class Localizer : public TelemetrySender {
    public:
        double velX;
        double velY;
        double posX;
        double posY;
        double heading;

    public:
        Localizer() : velX(0), velY(0), posX(0), posY(0), heading(0) {}

        void update(Roomba * roomba) {
            // auto t = roomba->gyro->getSamplingPeriod();
            // auto * values = roomba->gyro->getValues();
            // double dx = values[0] * t;
            // double dy = values[1] * t;

            // velX += dx;
            // velY += dy;

            heading = roomba->imu->getRollPitchYaw()[2];

            auto * gpsV = roomba->gps->getValues();
            posX = gpsV[0];
            posY = gpsV[1];

            // if (roomba->leftMotor->getVelocity() == 0
            //     && roomba->rightMotor->getVelocity() == 0) {
            //     velX = 0;
            //     velY = 0;
            // }
        }

        json getTelemetry() override {
            return json {
                {"loc",
                 {
                     {"heading", heading},
                     {"pos",
                      {
                          {"x", posX},
                          {"y", posY},
                      }},
                     {"vel",
                      {
                          {"x", velX},
                          {"y", velY},
                      }},
                 }},
            };
        }
    };
}
