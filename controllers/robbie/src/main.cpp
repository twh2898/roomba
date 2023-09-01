#include "robbie/Planning.hpp"
#include "robbie/World.hpp"
#include "robbie/util/Config.hpp"
#include "robbie/util/Telemetry.hpp"
#include "robbie/util/log.hpp"

using namespace robbie;

#define TIME_STEP 64

int main() {
    Logging::init_logging(spdlog::level::debug);
    Logging::Main->debug("Logging enabled");

    Config config;
    try {
        config = Config::fromFile("config.json");
    }
    catch (ConfigLoadException & e) {
        auto what = e.what();
        Logging::Main->error("Failed to load config: {}", what);
        return EXIT_FAILURE;
    }

    Telemetry tel(config.telemetry.port, config.telemetry.address);

    auto pidConfig = config.pid;
    // PID pid(-1, 1, pidConfig.p, pidConfig.i, pidConfig.d);

    Robbie robbie;
    robbie.platform.enable(TIME_STEP);
    robbie.mc.setSpeed(pidConfig.speed);

    WorldModel world;

    PathPlanning planner(robbie);

    Logging::Main->debug("Initialization complete");

    robbie.step(TIME_STEP);
    robbie.mc.setTarget(robbie.local.getHeading());
    auto h = robbie.local.getHeading();
    Logging::Main->debug("Starting heading is {}", h);

    planner.startUndock();

    while (robbie.step(TIME_STEP) != -1) {
        planner.update();
        tel.send(&robbie);
        tel.send(&planner);
    }

    return 0;
}
