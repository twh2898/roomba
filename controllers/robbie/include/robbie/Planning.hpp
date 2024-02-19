#pragma once

#include <vector>

#include "base/Robbie.hpp"
#include "util/Telemetry.hpp"
#include "util/XY.hpp"

namespace robbie {
    using std::vector;

    class PathPlanning : public TelemetrySender {
        enum Mode {
            FOLLOW = 0,
            TURN,
            GRID_SEARCH,
            REVERSE,
            REVERSE_TURN,
            UNDOCK,
            UNDOCK_TURN,
        } mode;

        Platform::Ptr platform;
        Localizer::Ptr local;
        MotionControl::Ptr mc;

        XY target;
        double zoneSize;

        double dist;

        vector<XY> path;
        int index;

        double reverseTime;

    public:
        PathPlanning(const Robbie::Ptr & robbie);

        void setPath(vector<XY> & newPath);

        double getZoneSize() const;

        void setZoneSize(double size);

        XY getTarget() const;

        void setTarget(XY newTarget);

        void startUndock();

        void startUndockTurn();

        void startReverse();

        void startReverseTurn();

        void startGridSearch();

        void update();

        json getTelemetry() const override;
    };
}
