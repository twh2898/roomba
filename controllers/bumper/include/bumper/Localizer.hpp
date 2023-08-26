#pragma once

#include <webots/Accelerometer.hpp>
#include <webots/Motor.hpp>

#include "Platform.hpp"
#include "Telemetry.hpp"
#include "XY.hpp"

namespace roomba {
    using webots::Accelerometer;
    using webots::Motor;

    class Localizer : public TelemetrySender {
    public:
        XY vel;
        XY pos;
        double heading;

    public:
        Localizer();

        void update(Roomba * roomba);

        json getTelemetry() const override;
    };
}
