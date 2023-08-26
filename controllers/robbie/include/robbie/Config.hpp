#pragma once

#include <map>
#include <string>

#include "json.hpp"

namespace robbie {
    using json = nlohmann::json;

    using std::string;
    using std::map;

    struct TelemetryConfig {
        int port;
        string address;
    };

    struct PIDConfig {
        double p;
        double i;
        double d;
        double speed;
    };

    struct Config {
        json data;

        TelemetryConfig telemetry;
        string pidMode;
        bool tuneMode;
        map<string, PIDConfig> pid;

        PIDConfig & getPID() const;

        static Config fromFile(const string & file);
    };
}
