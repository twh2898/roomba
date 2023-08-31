#include "robbie/base/Localizer.hpp"

namespace robbie {
    Localizer::Localizer(Platform & platform)
        : platform(platform), twist(0), heading(0), vel(0), pos(0, 0) {}

    double Localizer::getTwist() const {
        return twist;
    }

    double Localizer::getHeading() const {
        return heading;
    }

    double Localizer::getVelocity() const {
        return vel;
    }

    XY Localizer::getPosition() const {
        return pos;
    }

    void Localizer::update() {
        twist = platform.gyro->getValues()[2];

        heading = platform.imu->getRollPitchYaw()[2];

        auto * gpsV = platform.gps->getValues();
        XY newPos(gpsV[0], gpsV[1]);

        vel = pos.dist(newPos) / platform.dt();

        pos = newPos;
    }

    json Localizer::getTelemetry() const {
        return json {
            {"loc",
             {
                 {"twist", twist},
                 {"heading", heading},
                 {"velocity", vel},
                 {"pos",
                  {
                      {"x", pos.x},
                      {"y", pos.y},
                  }},
             }},
        };
    }
}
