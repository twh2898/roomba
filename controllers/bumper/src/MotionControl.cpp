#include "bumper/MotionControl.hpp"

#include <algorithm>

namespace roomba {
    using std::clamp;

    MotionControl::MotionControl(PID pid, Mode mode)
        : speed(20), targetHeading(0), pid(pid), mode(mode), steer(0), drive(0) {}

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

    void MotionControl::update(Roomba * roomba, Localizer * local) {
        double leftSpeed = 0;
        double rightSpeed = 0;

        auto t = roomba->getSamplingPeriod();
        double dt = t / 1000.0;
        double yaw = local->heading;

        if (mode == HEADING) {
            double e = targetHeading - yaw;
            if (e < -M_PI)
                e = -(e - M_PI);
            else if (e > M_PI)
                e = -(e - M_PI);

            double twist = pid.calculate(dt, 0, -e);
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
                 {"target", targetHeading},
                 {"steer", steer},
                 {"drive", drive},
                 {"pid", pid.getTelemetry()},
             }},
        };
    }
}