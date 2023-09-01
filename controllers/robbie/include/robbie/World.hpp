#pragma once

#include "util/Telemetry.hpp"

namespace robbie {
    class WorldModel : public TelemetrySender {
    public:
        json getTelemetry() const override;
    };
}
