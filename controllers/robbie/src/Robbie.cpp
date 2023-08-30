#include "robbie/Robbie.hpp"

namespace robbie {
    Robbie::Robbie() : platform(), local(platform), mc(platform, local) {}

    int Robbie::step(int duration) {
        int status = platform.step(duration);
        local.update();
        mc.update();
        return status;
    }

    json Robbie::getTelemetry() const {
        json res;
        res.update(platform.getTelemetry());
        res.update(local.getTelemetry());
        res.update(mc.getTelemetry());
        return res;
    }
}
