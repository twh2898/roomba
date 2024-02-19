#include "robbie/base/MotionControl.hpp"

#include <algorithm>
#include <cmath>

#include "robbie/util/log.hpp"

namespace robbie {
    using std::clamp;

    MotionControl::MotionControl(const Platform::Ptr & platform,
                                 const Localizer::Ptr & local,
                                 Mode mode)
        : platform(platform),
          local(local),
          speed(20),
          targetHeading(0),
          steerPID(-1, 1, 7, 0.01, 0.1),
          mode(mode),
          steer(0),
          drive(0) {}

    void MotionControl::updateMotors() {
        double leftSpeed = speed * clamp(drive - steer, -1.0, 1.0);
        double rightSpeed = speed * clamp(drive + steer, -1.0, 1.0);

        platform->leftMotor->setVelocity(leftSpeed);
        platform->rightMotor->setVelocity(rightSpeed);
    }

    MotionControl::Mode MotionControl::getMode() const {
        return mode;
    }

    void MotionControl::setMode(Mode val) {
        int i = val;
        Logging::MC->debug("Switch mode to {}", i);
        mode = val;
    }

    double MotionControl::getSpeed() const {
        return speed;
    }

    void MotionControl::setSpeed(double val) {
        speed = val;
    }

    // MANUAL MODE

    double MotionControl::getSteer() const {
        return steer;
    }

    void MotionControl::setSteer(double val) {
        if (mode == TARGET) {
            Logging::MC->warning("MotionControl::setSteer called in TARGET mode");
            return;
        }

        steer = val;
        updateMotors();
    }

    double MotionControl::getDrive() const {
        return drive;
    }

    void MotionControl::setDrive(double val) {
        if (mode == TARGET) {
            Logging::MC->warning("MotionControl::setDrive called in TARGET mode");
            return;
        }

        drive = clamp(val, -1.0, 1.0);
        updateMotors();
    }

    // TARGET MODE

    double MotionControl::getTarget() const {
        return targetHeading;
    }

    void MotionControl::setTarget(double val) {
        while (val > M_PI) val -= M_PI;
        while (val < -M_PI) val += M_PI;
        targetHeading = val;
    }

    void MotionControl::update() {
        if (mode == MANUAL)
            return;

        auto t = platform->getSamplingPeriod();
        double dt = t / 1000.0;

        if (mode == TARGET) {
            double e = targetHeading - local->getHeading();
            if (e < -M_PI)
                e = -(e - M_PI);
            else if (e > M_PI)
                e = -(e - M_PI);

            double twist = steerPID.calculate(dt, 0, -e);
            setSteer(twist);
        }

        updateMotors();
    }

    // Telemetry

    json MotionControl::getTelemetry() const {
        return json {
            {"mc",
             {
                 {"mode", mode},
                 {"target", targetHeading},
                 {"steer", steer},
                 {"drive", drive},
                 {"pid", steerPID.getTelemetry()},
             }},
        };
    }
}
