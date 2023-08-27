#include "robbie/Config.hpp"

#include <fstream>

namespace robbie {
    using std::fstream;

    Config::Config() {}

    static json testKey(string key, json data) {
        if (data.contains(key))
            return data["key"];
        else
            throw ConfigLoadException("Missing key " + key);
    }

    Config Config::fromFile(const string & file) {
        fstream f(file);
        json data = json::parse(f);

        auto telemConfig = testKey("telemetry", data);
        TelemetryConfig telem {
            port : testKey("port", telemConfig),
            address : testKey("address", telemConfig),
        };

        auto pidConfig = testKey("pid", data);
        PIDConfig pid {
            p : testKey("p", pidConfig),
            i : testKey("i", pidConfig),
            d : testKey("d", pidConfig),
            speed : testKey("speed", pidConfig),
        };

        bool tuneMode = testKey("tuneMode", data);

        auto targetConfig = testKey("target", data);
        TargetConfig target {
            heading : testKey("heading", targetConfig),
            size : testKey("size", targetConfig),
        };

        return Config {
            data : data,
            telemetry : telem,
            pid : pid,
            tuneMode : tuneMode,
            target : target,
        };
    }
}
