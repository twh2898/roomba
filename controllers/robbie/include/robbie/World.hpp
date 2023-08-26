#pragma once

#include "Localizer.hpp"
#include "Platform.hpp"
#include "Telemetry.hpp"

namespace robbie {
    class WorldModel : public TelemetrySender {
    public:
        json getTelemetry() const override;
    };
}
