#include "bumper/World.hpp"

namespace robbie {

    json World::getTelemetry() const {
        return json {
            {"world", {}},
        };
    }
}
