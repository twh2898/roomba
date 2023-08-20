#pragma once

#include <cmath>
#include <webots/Accelerometer.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>

#include "Telemetry.hpp"

namespace roomba {
    using namespace webots;

    class Roomba : public TelemetrySender {
    public:
        Robot * robot;
        Motor * leftMotor;
        Motor * rightMotor;
        Accelerometer * accel;
        Gyro * gyro;
        GPS * gps;
        InertialUnit * imu;
        PositionSensor * leftEncoder;
        PositionSensor * rightEncoder;
        TouchSensor * bumper;

    public:
        Roomba(Robot * robot);

        bool stopped() const;

        int getSamplingPeriod() const;

        void enable(int samplingPeriod);

        void disable();

        json getTelemetry() const override;
    };
}
