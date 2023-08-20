#include "bumper/Planning.hpp"

#include <cmath>

namespace roomba {
    using std::abs;
    using std::sqrt;
    using std::atan2;

    PathPlanning::PathPlanning(double x, double y, double zoneSize)
        : target(x, y), zoneSize(zoneSize), index(0), mode(TURN) {}

    void PathPlanning::setPath(vector<XY> & newPath) {
        path = newPath;
        index = 1;
        mode = TURN;
        setTarget(path[0]);
    }

    double PathPlanning::getZoneSize() const {
        return zoneSize;
    }

    void PathPlanning::setZoneSize(double size) {
        zoneSize = size;
    }

    XY PathPlanning::getTarget() const {
        return target;
    }

    void PathPlanning::setTarget(XY newTarget) {
        target = newTarget;
    }

    void PathPlanning::update(Roomba * roomba, Localizer * local, MotionControl * mc) {
        double dx = target.x - local->pos.x;
        double dy = target.y - local->pos.y;
        dist = abs(sqrt(dx * dx + dy * dy));

        double heading = atan2(dy, dx);
        mc->setTarget(heading);

        if (mode == TURN) {
            mc->setDrive(0);
            if (roomba->stopped())
                mode = FOLLOW;
            return;
        }
        else if (dist < zoneSize) {
            if (index < path.size()) {
                setTarget(path[index++]);
                mode = TURN;
            }
            else {
                mc->setDrive(0);
            }
            return;
        }

        mc->setDrive(1.0);
    }

    json PathPlanning::getTelemetry() const {
        return json {
            {"planning",
             {
                 {"target",
                  {
                      {"x", target.x},
                      {"y", target.y},
                  }},
                 {"dist", dist},
             }},
        };
    }
}
