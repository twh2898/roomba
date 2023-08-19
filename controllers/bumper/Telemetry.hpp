#pragma once

#include <string>

#include "json.hpp"
#include "simple_cpp_sockets.h"

namespace roomba {
    using json = nlohmann::json;

    using std::string;

    /**
     * @brief Interface for classes to send data though Telemetry.
     */
    class TelemetrySender {
    public:
        /**
         * @brief Get a json object with telemetry data.
         *
         * @return json telemetry object
         */
        virtual json getTelemetry() const = 0;
    };

    /**
     * @brief UDP Telemetry Broadcaster.
     */
    class Telemetry {
        UDPClient udp;

    public:
        /**
         * @brief Connect to Telemetry
         *
         * @param udpPort the UDP telemetry port
         * @param udpAddress the UDP telemetry network address
         */
        Telemetry(int udpPort, string udpAddress) : udp(udpPort, udpAddress) {}

        /**
         * @brief Send arbitrary telemetry data
         *
         * @param data json telemetry object
         */
        void send(json data) {
            udp.send_message(data.dump());
        }

        /**
         * @brief Send telemetry from a TelemetrySender via getTelemetry().
         *
         * @param sender the TelemetrySender
         */
        void send(TelemetrySender * sender) {
            send(sender->getTelemetry());
        }
    };
}
