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
            MANUAL = 0,
            TARGET,
        };

    private:
        Platform & platform;
        Localizer & local;

        double speed;
        double targetHeading;
        PID steerPID;
        Mode mode;

        double steer;
        double drive;

        /// Update for MANUAL mode
        void updateMotors();

    public:
        MotionControl(Platform & platform,
                      Localizer & local,
                      PID steerPID,
                      Mode mode = MANUAL);

        Mode getMode() const;

        void setMode(Mode val);

        double getSpeed() const;

        void setSpeed(double val);

        // MANUAL MODE

        double getSteer() const;

        void setSteer(double val);

        double getDrive() const;

        void setDrive(double val);

        // TARGET MODE

        double getTarget() const;

        void setTarget(double val);

        void update();

        // Telemetry

        json getTelemetry() const override;
    };
}
