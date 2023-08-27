#include "robbie/Robbie.hpp"

namespace robbie {
    Robbie::Robbie() : platform(), local() {}

    int Robbie::step(int duration) {
        int status = platform.step(duration);
        local.update(platform);
        return status;
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
