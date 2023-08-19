#pragma once

#include <string>

#include "json.hpp"
#include "simple_cpp_sockets.h"

namespace roomba {
    using json = nlohmann::json;

    using std::string;

    class TelemetrySender {
    public:
        virtual json getTelemetry() const = 0;
    };

    class Telemetry {
        UDPClient udp;

    public:
        Telemetry(int udpPort, string udpAddress) : udp(udpPort, udpAddress) {}

        void send(json data) {
            udp.send_message(data.dump());
        }

        void send(TelemetrySender * sender) {
            send(sender->getTelemetry());
        }
    };
}
