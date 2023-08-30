#include "robbie/MotionControl.hpp"

#include <algorithm>

namespace robbie {
    using std::clamp;

    MotionControl::MotionControl(Platform & platform,
                                 Localizer & local,
                                 PID steerPID,
                                 Mode mode)
        : platform(platform),
          local(local),
          speed(20),
          targetHeading(0),
          steerPID(steerPID),
          mode(mode),
          steer(0),
          drive(0) {}

    void MotionControl::updateMotors() {
        double leftSpeed = speed * clamp(drive - steer, -1.0, 1.0);
        double rightSpeed = speed * clamp(drive + steer, -1.0, 1.0);

        platform.leftMotor->setVelocity(leftSpeed);
        platform.rightMotor->setVelocity(rightSpeed);
    }

    MotionControl::Mode MotionControl::getMode() const {
        return mode;
    }

    void MotionControl::setMode(Mode val) {
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
        if (mode == MANUAL) {
            steer = val;
            updateMotors();
        }
    }

    double MotionControl::getDrive() const {
        return drive;
    }

    void MotionControl::setDrive(double val) {
        if (mode == MANUAL) {
            drive = clamp(val, -1.0, 1.0);
            updateMotors();
        }
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

        auto t = platform.getSamplingPeriod();
        double dt = t / 1000.0;

        if (mode == TARGET) {
            double e = targetHeading - local.getHeading();
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
                 {"mode", (mode == MANUAL ? 0 : 1)},
                 {"target", targetHeading},
                 {"steer", steer},
                 {"drive", drive},
                 {"pid", steerPID.getTelemetry()},
             }},
        };
    }
}
