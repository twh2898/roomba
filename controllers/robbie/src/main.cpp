#include "robbie/Planning.hpp"
#include "robbie/World.hpp"
#include "robbie/util/Config.hpp"
#include "robbie/util/Profiler.hpp"
#include "robbie/util/Telemetry.hpp"
#include "robbie/util/log.hpp"

using namespace robbie;

#include <stdexcept>

#define TIME_STEP 64

int main() {
    Logging::init_logging(spdlog::level::trace);
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

    Robbie::Ptr robbie = make_shared<Robbie>();
    robbie->platform->enable(TIME_STEP);
    robbie->mc->setSpeed(config.pid.speed);

    WorldModel world;

    PathPlanning planner(robbie);

    Logging::Main->info("Initialization complete");

    robbie->step(TIME_STEP);
    robbie->mc->setTarget(robbie->local->getHeading());
    auto h = robbie->local->getHeading();
    Logging::Main->debug("Starting heading is {}", h);

    planner.startUndock();

    Profiler prof;

    R_DEF_CLOCK(prof, clkMain, "main");
    R_DEF_CLOCK(prof, clkSim, "simulation");
    R_DEF_CLOCK(prof, clkPlan, "planner");
    R_DEF_CLOCK(prof, clkTelem, "telemetry");

    clkSim->reset();
    while (robbie->step(TIME_STEP) != -1) {
        clkSim->tick();

        R_PROFILE_STEP(clkPlan, planner.update());

        R_PROFILE_STEP(clkTelem, {
            tel.send(robbie.get());
            tel.send(&planner);
        });

        clkMain->tick();

        json profile {
            {"profile", prof.getTelemetry()},
        };
        tel.send(profile);

        clkSim->reset();
    }

    return 0;
}
