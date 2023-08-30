#pragma once

#include "Localizer.hpp"
#include "MotionControl.hpp"
#include "PID.hpp"
#include "Platform.hpp"
#include "Telemetry.hpp"

namespace robbie {
    class Robbie : public TelemetrySender {
    public:
        Platform platform;
        Localizer local;
        MotionControl mc;

        Robbie(PID steerPID);

        int step(int duration);

        json getTelemetry() const override;
    };
}
