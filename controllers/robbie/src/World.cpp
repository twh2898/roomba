#include "robbie/World.hpp"

namespace robbie {

    json World::getTelemetry() const {
        return json {
            {"world", {}},
        };
    }
}
