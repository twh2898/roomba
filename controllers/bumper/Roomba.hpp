#pragma once

#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>

namespace roomba {
    using namespace webots;

    class Roomba {
    public:
        Robot * robot;
        Motor * leftMotor;
        Motor * rightMotor;
        Accelerometer * accel;
        Gyro * gyro;
        PositionSensor * leftEncoder;
        PositionSensor * rightEncoder;
        TouchSensor * bumper;

    public:
        Roomba(Robot * robot) : robot(robot) {
            bumper = robot->getTouchSensor("bumper");
            accel = robot->getAccelerometer("accel");
            gyro = robot->getGyro("gyro");

            leftMotor = robot->getMotor("left wheel motor");
            rightMotor = robot->getMotor("right wheel motor");

            leftMotor->setPosition(INFINITY);
            rightMotor->setPosition(INFINITY);

            leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0);

            leftEncoder = robot->getPositionSensor("left wheel sensor");
            rightEncoder = robot->getPositionSensor("right wheel sensor");
        }

        void enable(int timeStep) {
            bumper->enable(timeStep);
            accel->enable(timeStep);
            gyro->enable(timeStep);
            leftEncoder->enable(timeStep);
            rightEncoder->enable(timeStep);
        }

        void disable() {
            bumper->disable();
            accel->disable();
            gyro->disable();
            leftEncoder->disable();
            rightEncoder->disable();
        }
    };

}
