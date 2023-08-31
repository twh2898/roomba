#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>
using namespace webots;

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
using namespace std;

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "robbie/Planning.hpp"
#include "robbie/World.hpp"
#include "robbie/base/Localizer.hpp"
#include "robbie/base/MotionControl.hpp"
#include "robbie/base/Platform.hpp"
#include "robbie/base/Robbie.hpp"
#include "robbie/util/Config.hpp"
#include "robbie/util/PID.hpp"
#include "robbie/util/Telemetry.hpp"
#include "robbie/util/log.hpp"

using namespace robbie;

#define TIME_STEP 64

int main() {
    spdlog::set_level(spdlog::level::debug);
    Logging::init_logging();
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
