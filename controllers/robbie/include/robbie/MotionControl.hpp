#pragma once

#include <cmath>

#include "Localizer.hpp"
#include "PID.hpp"
#include "Platform.hpp"
#include "Telemetry.hpp"

namespace robbie {
    class MotionControl : public TelemetrySender {
    public:
        enum Mode {
            MANUAL,
            HEADING,
        };

    private:
        Platform & platform;
        Localizer & local;

        double speed;
        double heading; // Current heading as of last update
        double targetHeading;
        PID steerPID;
        Mode mode;

        double steer;
        double drive;

    public:
        MotionControl(Platform & platform,
                      Localizer & local,
                      PID steerPID,
                      Mode mode = MANUAL);

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

        double headingDiff() const;

        void update();

        json getTelemetry() const override;
    };
}
