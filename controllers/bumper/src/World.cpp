#include "bumper/World.hpp"

namespace roomba {

    json World::getTelemetry() const {
        return json {
            {"world", {}},
        };
    }
}
