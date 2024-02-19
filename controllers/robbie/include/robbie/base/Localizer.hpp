#pragma once

#include <memory>

#include "Platform.hpp"
#include "robbie/util/Telemetry.hpp"
#include "robbie/util/XY.hpp"

namespace robbie {
    using webots::Accelerometer;
    using webots::Motor;
    using std::shared_ptr;

    class Localizer : public TelemetrySender {
    public:
        using Ptr = shared_ptr<Localizer>;
        using ConstPtr = const shared_ptr<Localizer>;

    private:
        Platform::Ptr platform;

        double twist;
        double heading;
        double vel;
        XY pos;

    public:
        Localizer(Platform::Ptr & platform);

        double getTwist() const;

        double getHeading() const;

        double getVelocity() const;

        XY getPosition() const;

        void update();

        json getTelemetry() const override;
    };
}
