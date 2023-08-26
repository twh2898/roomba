#include "bumper/Planning.hpp"

#include <cmath>

#include "bumper/log.hpp"

namespace robbie {
    using std::abs;
    using std::sqrt;
    using std::atan2;

    PathPlanning::PathPlanning(double x, double y, double zoneSize)
        : target(x, y),
          zoneSize(zoneSize),
          index(0),
          mode(GRID_SEARCH),
          reverseTime(0.0) {}

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

    void PathPlanning::startUndock() {
        Logging::Core->debug("Planning mode switched to UNDOCK");
        mode = UNDOCK;
        reverseTime = 2.0;
    }

    void PathPlanning::startReverse() {
        Logging::Core->debug("Planning mode switch to REVERSE");
        mode = REVERSE;
        reverseTime = 1.0;
    }

    void PathPlanning::startReverseTurn() {
        Logging::Core->debug("Planning mode switch to REVERSE_TURN");
        mode = REVERSE_TURN;
    }

    void PathPlanning::startGridSearch() {
        Logging::Core->debug("Planning mode switch to GRID_SEARCH");
        mode = GRID_SEARCH;
    }

    void PathPlanning::update(Platform * roomba, Localizer * local, MotionControl * mc) {
        double dt = roomba->dt();

        if (mode == GRID_SEARCH) {
            bool bump = roomba->bumper->getValue() == 1.0;
            if (bump) {
                startReverse();
                return;
            }

            mc->setDrive(1.0);
            return;
        }
        else if (mode == REVERSE || mode == UNDOCK) {
            if (reverseTime <= 0.0) {
                if (mode == REVERSE) {
                    startReverseTurn();
                    mc->setTarget(local->heading + M_PI / 3);
                }
                else {
                    mode = UNDOCK_TURN;
                    mc->setTarget(local->heading + M_PI);
                }
                return;
            }

            mc->setDrive(-0.5);
            reverseTime -= dt;

            return;
        }
        else if (mode == REVERSE_TURN || mode == UNDOCK_TURN) {
            if (abs(local->heading - mc->getTarget()) < 0.01) {
                startGridSearch();
            }
            else {
                mc->setDrive(0.0);
            }
            return;
        }

        double dx = target.x - local->pos.x;
        double dy = target.y - local->pos.y;
        dist = abs(sqrt(dx * dx + dy * dy));

        double heading = atan2(dy, dx);
        mc->setTarget(heading);

        if (mode == TURN) {
            mc->setDrive(0);
            if (abs(local->heading - heading) < 0.01) {
                Logging::Core->debug("Planning mode switch to FOLLOW");
                mode = FOLLOW;
            }
            return;
        }
        else if (dist < zoneSize) {
            if (index < path.size()) {
                Logging::Core->debug("Planning mode switch to TURN");
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
                 {"reverseTime", reverseTime},
                 {"mode", mode},
                 {"dist", dist},
             }},
        };
    }
}
