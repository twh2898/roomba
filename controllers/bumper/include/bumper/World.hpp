#pragma once

#include "Localizer.hpp"
#include "Roomba.hpp"
#include "Telemetry.hpp"

namespace roomba {
    class World : public TelemetrySender {
    public:
        json getTelemetry() const override;
    };
}
