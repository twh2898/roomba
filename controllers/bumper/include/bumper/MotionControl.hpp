#pragma once

#include <cmath>

#include "Localizer.hpp"
#include "PID.hpp"
#include "Roomba.hpp"
#include "Telemetry.hpp"

namespace roomba {
    class MotionControl : public TelemetrySender {
    public:
        enum Mode {
            MANUAL,
            HEADING,
        };

    private:
        double speed;
        double targetHeading;
        PID pid;
        Mode mode;

        double steer;
        double drive;

    public:
        MotionControl(PID pid, Mode mode = MANUAL);

        Mode getMode() const;

        void setMode(Mode val);

        double getSpeed() const;

        void setSpeed(double val);

        double getTarget() const;

        void setTarget(double val);

        double getSteer() const;

        void setSteer(double val);

        double getDrive() const;

        void setDrive(double val);

        void update(Roomba * roomba, Localizer * local);

        json getTelemetry() const override;
    };
}