#include "bumper/Localizer.hpp"

namespace roomba {
    Localizer::Localizer() : vel(0, 0), pos(0, 0), heading(0) {}

    void Localizer::update(Platform * roomba) {
        heading = roomba->imu->getRollPitchYaw()[2];

        auto * gpsV = roomba->gps->getValues();
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
