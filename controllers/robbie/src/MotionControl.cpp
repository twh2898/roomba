#include "robbie/MotionControl.hpp"

#include <algorithm>

namespace robbie {
    using std::clamp;

    MotionControl::MotionControl(PID steerPID, Mode mode)
        : speed(20),
          heading(0),
          targetHeading(0),
          steerPID(steerPID),
          mode(mode),
          steer(0),
          drive(0) {}

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

    double MotionControl::getTarget() const {
        return targetHeading;
    }

    void MotionControl::setTarget(double val) {
        while (val > M_PI) val -= M_PI;
        while (val < -M_PI) val += M_PI;
        targetHeading = val;
    }

    double MotionControl::getSteer() const {
        return steer;
    }

    void MotionControl::setSteer(double val) {
        steer = val;
    }

    double MotionControl::getDrive() const {
        return drive;
    }

    void MotionControl::setDrive(double val) {
        drive = clamp(val, -1.0, 1.0);
    }

    double MotionControl::headingDiff() const {
        return targetHeading - heading;
    }

    void MotionControl::update(Platform * roomba, Localizer * local) {
        double leftSpeed = 0;
        double rightSpeed = 0;

        auto t = roomba->getSamplingPeriod();
        double dt = t / 1000.0;
        heading = local->heading;

        if (mode == HEADING) {
            double e = targetHeading - heading;
            if (e < -M_PI)
                e = -(e - M_PI);
            else if (e > M_PI)
                e = -(e - M_PI);

            double twist = steerPID.calculate(dt, 0, -e);
            setSteer(twist);
        }

        leftSpeed = speed * clamp(drive - steer, -1.0, 1.0);
        rightSpeed = speed * clamp(drive + steer, -1.0, 1.0);

        roomba->leftMotor->setVelocity(leftSpeed);
        roomba->rightMotor->setVelocity(rightSpeed);
    }

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
