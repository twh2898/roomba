#pragma once

#include <memory>

#include "Localizer.hpp"
#include "Platform.hpp"
#include "robbie/util/PID.hpp"
#include "robbie/util/Telemetry.hpp"

namespace robbie {
    using std::shared_ptr;

    class MotionControl : public TelemetrySender {
    public:
        using Ptr = shared_ptr<MotionControl>;
        using ConstPtr = const shared_ptr<MotionControl>;

        enum Mode {
            MANUAL = 0,
            TARGET,
        };

    private:
        Platform::Ptr platform;
        Localizer::Ptr local;

        Mode mode;
        double speed;

        // MANUAL MODE
        double steer;
        double drive;

        /// Update for MANUAL mode
        void updateMotors();

        // TARGET MODE

        double targetHeading;
        PID steerPID;

    public:
        MotionControl(const Platform::Ptr & platform,
                      const Localizer::Ptr & local,
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
