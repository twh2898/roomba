#pragma once

#include "Localizer.hpp"
#include "Platform.hpp"
#include "Telemetry.hpp"

namespace robbie {
    class World : public TelemetrySender {
    public:
        json getTelemetry() const override;
    };
}
