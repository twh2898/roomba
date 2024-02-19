#pragma once

#include <memory>

#include "Localizer.hpp"
#include "MotionControl.hpp"
#include "Platform.hpp"
#include "robbie/util/Telemetry.hpp"

namespace robbie {
    using std::shared_ptr;

    class Robbie : public TelemetrySender {
    public:
        using Ptr = shared_ptr<Robbie>;
        using ConstPtr = const shared_ptr<Robbie>;

        Platform::Ptr platform;
        Localizer::Ptr local;
        MotionControl::Ptr mc;

        Robbie();

        int step(int duration);

        json getTelemetry() const override;
    };
}
