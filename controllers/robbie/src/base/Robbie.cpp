#include "robbie/base/Robbie.hpp"

namespace robbie {
    using std::make_shared;

    Robbie::Robbie()
        : platform(make_shared<Platform>()),
          local(make_shared<Localizer>(platform)),
          mc(make_shared<MotionControl>(platform, local)) {}

    int Robbie::step(int duration) {
        int status = platform->step(duration);
        if (status != -1) {
            local->update();
            mc->update();
        }
        return status;
    }

    json Robbie::getTelemetry() const {
        json res;
        res.update(platform->getTelemetry());
        res.update(local->getTelemetry());
        res.update(mc->getTelemetry());
        return res;
    }
}
