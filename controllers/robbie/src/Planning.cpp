#include "robbie/Planning.hpp"

#include <cmath>

#include "robbie/util/log.hpp"

namespace robbie {
    using std::abs;
    using std::sqrt;
    using std::atan2;

    PathPlanning::PathPlanning(const Robbie::Ptr & robbie)
        : platform(robbie->platform),
          local(robbie->local),
          mc(robbie->mc),
          target(0, 0),
          zoneSize(1),
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
        Logging::Planning->debug("Planning mode switched to UNDOCK");
        mode = UNDOCK;
        reverseTime = 2.0;
    }

    void PathPlanning::startUndockTurn() {
        Logging::Planning->debug("Planning mode switch to UNDOCK_TURN");
        mode = UNDOCK_TURN;
    }

    void PathPlanning::startReverse() {
        Logging::Planning->debug("Planning mode switch to REVERSE");
        mode = REVERSE;
        reverseTime = 1.0;
    }

    void PathPlanning::startReverseTurn() {
        Logging::Planning->debug("Planning mode switch to REVERSE_TURN");
        mode = REVERSE_TURN;
    }

    void PathPlanning::startGridSearch() {
        Logging::Planning->debug("Planning mode switch to GRID_SEARCH");
        mode = GRID_SEARCH;
    }

    void PathPlanning::update() {
        double dt = platform->dt();

        switch (mode) {
            case GRID_SEARCH: {
                bool bump = platform->bumper->getValue() == 1.0;
                if (bump) {
                    startReverse();
                }
                else {
                    mc->setDrive(1.0);
                }
            } break;
            case REVERSE:
            case UNDOCK: {
                if (reverseTime <= 0.0) {
                    if (mode == REVERSE) {
                        startReverseTurn();
                        mc->setTarget(local->getHeading() + M_PI / 3);
                    }
                    else {
                        startUndockTurn();
                        mc->setTarget(local->getHeading() + M_PI / 3);
                    }
                }
                else {
                    mc->setDrive(-0.5);
                    reverseTime -= dt;
                }
            } break;
            case REVERSE_TURN:
            case UNDOCK_TURN: {
                if (abs(local->getHeading() - mc->getTarget()) < 0.01) {
                    startGridSearch();
                }
                else {
                    mc->setDrive(0.0);
                }
            } break;
            default:
                Logging::Planning->warning("Reached a bad state mode={}",
                                           (int)mode);
                break;
        }

        return;

        auto delta = local->getPosition() - target;
        delta.normalize();
        dist = local->getPosition().dist(target);

        double heading = delta.angle();
        mc->setTarget(heading);

        if (mode == TURN) {
            mc->setDrive(0);
            if (abs(local->getHeading() - heading) < 0.01) {
                Logging::Planning->debug("Planning mode switch to FOLLOW");
                mode = FOLLOW;
            }
            return;
        }
        else if (dist < zoneSize) {
            if (index < path.size()) {
                Logging::Planning->debug("Planning mode switch to TURN");
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
