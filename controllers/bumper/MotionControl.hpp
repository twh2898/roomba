#pragma once

#include <cmath>

#include "PID.hpp"
#include "Roomba.hpp"

namespace roomba {

    inline double limit(double x, double a = -1, double b = 1) {
        if (x < a)
            x = a;
        else if (x > b)
            x = b;
        return x;
    }

    class MotionControl {
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
        MotionControl(Mode mode = MANUAL)
            : speed(20),
              targetHeading(0),
              pid(1, -1, 4, 0.0000018, 64),
              mode(mode),
              steer(0),
              drive(0) {}

        double currYaw = 0;

        Mode getMode() {
            return mode;
        }

        void setMode(Mode val) {
            mode = val;
        }

        double getTarget() {
            return targetHeading;
        }

        void setTarget(double target) {
            targetHeading = target;
        }

        double getSteer() {
            return steer;
        }

        void setSteer(double val) {
            steer = limit(val, -1, 1);
        }

        double getDrive() {
            return drive;
        }

        void setDrive(double val) {
            drive = limit(val, -1, 1);
        }

        void update(Roomba * roomba) {
            double leftSpeed = 0;
            double rightSpeed = 0;

            if (mode == HEADING) {
                auto t = roomba->imu->getSamplingPeriod();

                double yaw = roomba->imu->getRollPitchYaw()[2];
                currYaw = yaw;

                double e = targetHeading - roomba->imu->getRollPitchYaw()[2];
                if (e < -M_PI)
                    e = -(e - M_PI);
                else if (e > M_PI)
                    e = -(e - M_PI);

                double twist = pid.calculate(t, 0, -e);
                setSteer(twist);
            }

            leftSpeed = speed * limit(drive - steer, -1, 1);
            rightSpeed = speed * limit(drive + steer, -1, 1);

            roomba->leftMotor->setVelocity(leftSpeed);
            roomba->rightMotor->setVelocity(rightSpeed);
        }
    };
}
