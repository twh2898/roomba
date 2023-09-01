#pragma once

#include <arpa/inet.h>

#include <chrono>
#include <string>

#include "json.hpp"

namespace robbie {
    namespace chrono = std::chrono;
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
        int m_socket;
        sockaddr_in m_addr;

    public:
        /**
         * @brief Connect to Telemetry
         *
         * @param udpPort the UDP telemetry port
         * @param udpAddress the UDP telemetry network address
         *
         * @throw std::runtime_error
         */
        Telemetry(int udpPort, string udpAddress);

        /**
         * @brief Send arbitrary telemetry data
         *
         * @param data json telemetry object
         */
        void send(json data);

        /**
         * @brief Send telemetry from a TelemetrySender via getTelemetry().
         *
         * @param sender the TelemetrySender
         */
        void send(TelemetrySender * sender);
    };

    class Profiler {
    public:
        using clock = std::chrono::high_resolution_clock;

    private:
        clock::time_point lastPoint;

    public:
        Profiler() {
            reset();
        }

        void reset() {
            lastPoint = clock::now();
        }

        clock::duration tick() {
            auto now = clock::now();
            auto delta = now - lastPoint;
            lastPoint = now;
            return delta;
        }

        uint64_t tick_ns() {
            return chrono::duration_cast<chrono::nanoseconds>(tick()).count();
        }

        uint64_t tick_ms() {
            return chrono::duration_cast<chrono::milliseconds>(tick()).count();
        }

        double tick_s() {
            using seconds = chrono::duration<double>;
            return chrono::duration_cast<seconds>(tick()).count();
        }
    };
}
