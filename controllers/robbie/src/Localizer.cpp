#include "robbie/Localizer.hpp"

namespace robbie {
    Localizer::Localizer() : vel(0, 0), pos(0, 0), heading(0) {}

    void Localizer::update(Platform & platform) {
        heading = platform.imu->getRollPitchYaw()[2];

        auto * gpsV = platform.gps->getValues();
        pos.x = gpsV[0];
        pos.y = gpsV[1];
    }

    json Localizer::getTelemetry() const {
        return json {
            {"loc",
             {
                 {"heading", heading},
                 {"pos",
                  {
                      {"x", pos.x},
                      {"y", pos.y},
                  }},
                 {"vel",
                  {
                      {"x", vel.x},
                      {"y", vel.y},
                  }},
             }},
        };
    }
}
