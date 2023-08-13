#pragma once

#include <string>

#include "Roomba.hpp"
#include "json.hpp"
#include "simple_cpp_sockets.h"

namespace roomba {
    using namespace webots;
    using json = nlohmann::json;

    using std::string;

    class Telemetry {
        UDPClient udp;

    public:
        Telemetry(int udpPort, string udpAddress) : udp(udpPort, udpAddress) {}

        void sendCustom(json data) {
            udp.send_message(data.dump());
        }

        void send(Roomba * roomba) {
            json motorData = {
                {"left",
                 {
                     {"velocity", roomba->leftMotor->getVelocity()},
                     {"position", roomba->leftEncoder->getValue()},
                 }},
                {"right",
                 {
                     {"velocity", roomba->rightMotor->getVelocity()},
                     {"position", roomba->rightEncoder->getValue()},
                 }},
            };

            auto * accel = roomba->accel->getValues();
            json accelData = {
                {"x", accel[0]},
                {"y", accel[1]},
                {"z", accel[2]},
            };

            auto * gyro = roomba->gyro->getValues();
            json gyroData = {
                {"x", gyro[0]},
                {"y", gyro[1]},
                {"z", gyro[2]},
            };

            auto * gps = roomba->gps->getValues();
            json gpsData = {
                {"x", gps[0]},
                {"y", gps[1]},
                {"z", gps[2]},
            };

            auto * imu = roomba->imu->getRollPitchYaw();
            json imuData = {
                {"x", imu[0]},
                {"y", imu[1]},
                {"z", imu[2]},
            };

            json sensorData = {
                {"accel", accelData},
                {"gyro", gyroData},
                {"gps", gpsData},
                {"imu", imuData},
            };

            json data = {
                {"motor", motorData},
                {"sensors", sensorData},
                // {"local",
                //  {
                //      {"velocity",
                //       {
                //           {"x", local.velX},
                //           {"y", local.velY},
                //       }},
                //      {"position",
                //       {
                //           {"x", local.posX},
                //           {"y", local.posY},
                //       }},
                //  }},
                {"time", roomba->robot->getTime()},
            };
            udp.send_message(data.dump());
        }
    };
}
