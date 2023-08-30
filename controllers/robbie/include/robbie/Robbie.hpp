#pragma once

#include "Localizer.hpp"
#include "MotionControl.hpp"
#include "Platform.hpp"
#include "Telemetry.hpp"

namespace robbie {
    class Robbie : public TelemetrySender {
    public:
        Platform platform;
        Localizer local;
        MotionControl mc;

        Robbie();

        int step(int duration);

        json getTelemetry() const override;
    };
}
