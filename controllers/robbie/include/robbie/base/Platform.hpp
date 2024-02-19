#pragma once

#include <memory>
#include <webots/Accelerometer.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>

#include "robbie/util/Telemetry.hpp"

namespace robbie {
    using namespace webots;
    using std::shared_ptr;

    class Platform : public TelemetrySender {
        int lastStep;

    public:
        using Ptr = shared_ptr<Platform>;
        using ConstPtr = const shared_ptr<Platform>;

        const float radius = 0.155;

    public:
        Robot robot;
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
        Platform();

        int step(int duration);

        int getSamplingPeriod() const;

        double dt() const;

        void enable(int samplingPeriod);

        void disable();

        json getTelemetry() const override;
    };
}
