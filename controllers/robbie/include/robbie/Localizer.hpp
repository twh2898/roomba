#pragma once

#include <webots/Accelerometer.hpp>
#include <webots/Motor.hpp>

#include "Platform.hpp"
#include "Telemetry.hpp"
#include "XY.hpp"

namespace robbie {
    using webots::Accelerometer;
    using webots::Motor;

    class Robbie;

    class Localizer : public TelemetrySender {
        double twist;
        double heading;
        double vel;
        XY pos;

    public:
        Localizer();

        double getTwist() const;

        double getHeading() const;

        double getVelocity() const;

        XY getPosition() const;

        void update(Platform & platform);

        json getTelemetry() const override;
    };
}
