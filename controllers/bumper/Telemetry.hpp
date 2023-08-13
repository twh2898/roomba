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

        void send(Roomba * roomba) {
            auto * comp = roomba->accel->getValues();

            json accelData = {
                {"accel",
                 {
                     {"x", comp[0]},
                     {"y", comp[1]},
                     {"z", comp[2]},
                 }},
                {"left",
                 {
                     {"velocity", roomba->leftMotor->getVelocity()},
                 }},
                {"right",
                 {
                     {"velocity", roomba->rightMotor->getVelocity()},
                 }},
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
            udp.send_message(accelData.dump());
        }
    };
}
