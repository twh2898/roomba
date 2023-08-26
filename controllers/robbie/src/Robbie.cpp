#include "robbie/Robbie.hpp"

namespace robbie {
    Robbie::Robbie() : platform(), local() {}

    void Robbie::step(int duration) {
        platform.step(duration);
        local.update(platform);
    }

    json Robbie::getTelemetry() const {
        json res;
        res.update(platform.getTelemetry());
        res.update(local.getTelemetry());
        // res.update(world.getTelemetry());
        // res.update(planner.getTelemetry());
        // res.update(mc.getTelemetry());
        return res;
    }
}
