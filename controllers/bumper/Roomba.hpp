#pragma once

#include <webots/Accelerometer.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>

#include "Telemetry.hpp"

namespace roomba {
    using namespace webots;

    class Roomba : public TelemetrySender {
    public:
        Robot * robot;
        Motor * leftMotor;
        Motor * rightMotor;
        Accelerometer * accel;
        Gyro * gyro;
        GPS * gps;
        InertialUnit * imu;
        PositionSensor * leftEncoder;
        PositionSensor * rightEncoder;
        TouchSensor * bumper;

    public:
        Roomba(Robot * robot) : robot(robot) {
            bumper = robot->getTouchSensor("bumper");
            accel = robot->getAccelerometer("accel");
            gyro = robot->getGyro("gyro");
            gps = robot->getGPS("gps");
            imu = robot->getInertialUnit("imu");

            leftMotor = robot->getMotor("left wheel motor");
            rightMotor = robot->getMotor("right wheel motor");

            leftMotor->setPosition(INFINITY);
            rightMotor->setPosition(INFINITY);

            leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0);

            leftEncoder = robot->getPositionSensor("left wheel sensor");
            rightEncoder = robot->getPositionSensor("right wheel sensor");
        }

        int getSamplingPeriod() {
            return imu->getSamplingPeriod();
        }

        void enable(int samplingPeriod) {
            bumper->enable(samplingPeriod);
            accel->enable(samplingPeriod);
            gyro->enable(samplingPeriod);
            gps->enable(samplingPeriod);
            imu->enable(samplingPeriod);
            leftEncoder->enable(samplingPeriod);
            rightEncoder->enable(samplingPeriod);
        }

        void disable() {
            bumper->disable();
            accel->disable();
            gyro->disable();
            gps->disable();
            imu->disable();
            leftEncoder->disable();
            rightEncoder->disable();
        }

        json getTelemetry() override {
            json motorData = {
                {"left",
                 {
                     {"velocity", leftMotor->getVelocity()},
                     {"position", leftEncoder->getValue()},
                 }},
                {"right",
                 {
                     {"velocity", rightMotor->getVelocity()},
                     {"position", rightEncoder->getValue()},
                 }},
            };

            auto * accelV = accel->getValues();
            json accelData = {
                {"x", accelV[0]},
                {"y", accelV[1]},
                {"z", accelV[2]},
            };

            auto * gyroV = gyro->getValues();
            json gyroData = {
                {"x", gyroV[0]},
                {"y", gyroV[1]},
                {"z", gyroV[2]},
            };

            auto * gpsV = gps->getValues();
            json gpsData = {
                {"x", gpsV[0]},
                {"y", gpsV[1]},
                {"z", gpsV[2]},
            };

            auto * imuV = imu->getRollPitchYaw();
            json imuData = {
                {"x", imuV[0]},
                {"y", imuV[1]},
                {"z", imuV[2]},
            };

            json sensorData = {
                {"accel", accelData},
                {"gyro", gyroData},
                {"gps", gpsData},
                {"imu", imuData},
            };

            return json {
                {"motor", motorData},
                {"sensors", sensorData},
                {"time", robot->getTime()},
            };
        }
    };
}
