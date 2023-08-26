#pragma once

#include "Localizer.hpp"
// #include "MotionControl.hpp"
// #include "Planning.hpp"
#include "Platform.hpp"
#include "Telemetry.hpp"
// #include "World.hpp"

namespace robbie {
    class Robbie : public TelemetrySender {
    public:
        Platform platform;
        Localizer local;
        // WorldModel world;
        // PathPlanning planner;
        // MotionControl mc;

        Robbie();

        void step(int duration);

        json getTelemetry() const override;
    };
}
