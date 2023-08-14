#pragma once

#include "PID.hpp"
#include "Roomba.hpp"

namespace roomba {

    class MotionControl {
        double speed;
        double targetHeading;
        PID pid;

    public:
        MotionControl()
            : speed(5), targetHeading(0), pid(1, -1, 10, 0.0000589, 64) {}

        void setTarget(double target) {
            targetHeading = target;
        }

        void update(Roomba * roomba) {
            auto t = roomba->imu->getSamplingPeriod();

            double leftSpeed = 0;
            double rightSpeed = 0;

            double yaw = roomba->imu->getRollPitchYaw()[2];

            double e = targetHeading - roomba->imu->getRollPitchYaw()[2];
            if (e < -M_PI)
                e = -(e - M_PI);
            else if (e > M_PI)
                e = -(e - M_PI);

            double twist = pid.calculate(t, 0, -e);

            leftSpeed = -speed * twist;
            rightSpeed = speed * twist;

            roomba->leftMotor->setVelocity(leftSpeed);
            roomba->rightMotor->setVelocity(rightSpeed);
        }
    };
}
