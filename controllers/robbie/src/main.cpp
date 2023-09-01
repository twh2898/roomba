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

    Profiler prof, profPlan, profTelem, profSim;

    profSim.reset();
    while (robbie.step(TIME_STEP) != -1) {
        auto simDelta = profSim.tick_s();
        profPlan.reset();
        planner.update();
        auto planDelta = profPlan.tick_s();

        profTelem.reset();
        tel.send(&robbie);
        tel.send(&planner);
        auto telDelta = profTelem.tick_s();

        json profile {
            {"profile",
             {
                 {"sim", simDelta},
                 {"planner", planDelta},
                 {"telemetry", telDelta},
                 {"main", prof.tick_s()},
             }},
        };
        tel.send(profile);
        profSim.reset();
    }

    return 0;
}
