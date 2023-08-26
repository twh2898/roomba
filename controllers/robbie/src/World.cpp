#include "robbie/World.hpp"

namespace robbie {

    json WorldModel::getTelemetry() const {
        return json {
            {"world", {}},
        };
    }
}
