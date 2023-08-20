#pragma once

#include <vector>

#include "MotionControl.hpp"
#include "Roomba.hpp"
#include "Telemetry.hpp"
#include "XY.hpp"

namespace roomba {
    using std::vector;

    class PathPlanning : public TelemetrySender {
        enum Mode {
            FOLLOW,
            TURN,
        } mode;

        XY target;
        double zoneSize;

        double dist;

        vector<XY> path;
        int index;

    public:
        PathPlanning(double x = 1, double y = 1, double zoneSize = 1);

        void setPath(vector<XY> & newPath);

        double getZoneSize() const;

        void setZoneSize(double size);

        XY getTarget() const;

        void setTarget(XY newTarget);

        void update(Roomba * roomba, Localizer * local, MotionControl * mc);

        json getTelemetry() const override;
    };
}
