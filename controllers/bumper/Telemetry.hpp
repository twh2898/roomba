#pragma once

#include "Roomba.hpp"

namespace roomba {
    using namespace webots;

    class Telemetry {
        Roomba * roomba;

    public:
        Telemetry(Roomba * roomba) : roomba(roomba) {}

        void send() {}
    };
}
