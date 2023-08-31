#pragma once

#include "base/Localizer.hpp"
#include "base/Platform.hpp"
#include "util/Telemetry.hpp"

namespace robbie {
    class WorldModel : public TelemetrySender {
    public:
        json getTelemetry() const override;
    };
}
