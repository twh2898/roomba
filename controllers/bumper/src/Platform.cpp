#include "bumper/Platform.hpp"

namespace roomba {
    Platform::Platform(Robot * robot) : robot(robot) {
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

    bool Platform::stopped() const {
        return abs(leftMotor->getVelocity()) < 0.05
               && abs(rightMotor->getVelocity()) < 0.05;
    }

    int Platform::getSamplingPeriod() const {
        return imu->getSamplingPeriod();
    }

    void Platform::enable(int samplingPeriod) {
        bumper->enable(samplingPeriod);
        accel->enable(samplingPeriod);
        gyro->enable(samplingPeriod);
        gps->enable(samplingPeriod);
        imu->enable(samplingPeriod);
        leftEncoder->enable(samplingPeriod);
        rightEncoder->enable(samplingPeriod);
    }

    void Platform::disable() {
        bumper->disable();
        accel->disable();
        gyro->disable();
        gps->disable();
        imu->disable();
        leftEncoder->disable();
        rightEncoder->disable();
    }

    double Platform::dt() const {
        return (double)getSamplingPeriod() / 1000.0;
    }

    json Platform::getTelemetry() const {
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
            {"bumper", bumper->getValue()},
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
}
