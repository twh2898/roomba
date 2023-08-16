#pragma once

#include <cmath>
#include <iostream>
#include <vector>

#include "MotionControl.hpp"
#include "Roomba.hpp"
#include "Telemetry.hpp"

namespace roomba {
    using std::vector;

    struct XY {
        double x;
        double y;

        XY(double x = 0, double y = 0) : x(x), y(y) {}
    };

    class PathPlanning : public TelemetrySender {
        XY target;
        double zoneSize;

        double dist;

        vector<XY> path;
        int index;

    public:
        PathPlanning(double x = 1, double y = 1, double zoneSize = 1)
            : target(x, y), zoneSize(zoneSize), index(0) {}

        void setPath(vector<XY> & newPath) {
            path = newPath;
            index = 1;
            setTarget(path[0]);
        }

        double getZoneSize() const {
            return zoneSize;
        }

        void setZoneSize(double size) {
            zoneSize = size;
        }

        XY getTarget() const {
            return target;
        }

        void setTarget(XY newTarget) {
            target = newTarget;
        }

        void update(Roomba * roomba, Localizer * local, MotionControl * mc) {
            double dx = target.x - local->posX;
            double dy = target.y - local->posY;
            dist = abs(sqrt(dx * dx + dy * dy));

            if (dist < zoneSize) {
                if (index < path.size()) {
                    std::cout << "In zone " << index << std::endl;
                    setTarget(path[index++]);
                }
                else {
                    mc->setDrive(0);
                }
                return;
            }

            double heading = atan2(dy, dx);
            mc->setTarget(heading);
            mc->setDrive(1.0);
        }

        json getTelemetry() const override {
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
    };
}
