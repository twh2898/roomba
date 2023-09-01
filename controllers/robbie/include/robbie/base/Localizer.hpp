#pragma once

#include "Platform.hpp"
#include "robbie/util/Telemetry.hpp"
#include "robbie/util/XY.hpp"

namespace robbie {
    using webots::Accelerometer;
    using webots::Motor;

    class Robbie;

    class Localizer : public TelemetrySender {
        Platform & platform;

        double twist;
        double heading;
        double vel;
        XY pos;

    public:
        Localizer(Platform & platform);

        double getTwist() const;

        double getHeading() const;

        double getVelocity() const;

        XY getPosition() const;

        void update();

        json getTelemetry() const override;
    };
}
