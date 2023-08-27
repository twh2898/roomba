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

#include "robbie/Config.hpp"
#include "robbie/Localizer.hpp"
#include "robbie/MotionControl.hpp"
#include "robbie/PID.hpp"
#include "robbie/Planning.hpp"
#include "robbie/Platform.hpp"
#include "robbie/Robbie.hpp"
#include "robbie/Telemetry.hpp"
#include "robbie/World.hpp"
#include "robbie/log.hpp"

using namespace robbie;

#define TIME_STEP 64

namespace robbie::Logging {
    Logger::Ptr Core;
    Logger::Ptr Planning;
}

int main() {
    spdlog::set_level(spdlog::level::debug);
    Logging::Core = make_shared<Logging::Logger>("Core");
    Logging::Core->debug("Logging enabled");

    Logging::Planning = make_shared<Logging::Logger>("Planning");

    Config config;
    try {
        config = Config::fromFile("config.json");
    }
    catch (ConfigLoadException & e) {
        Logging::Core->critical("Failed to load config: {}", e.what());
    }

    Telemetry tel(config.telemetry.port, config.telemetry.address);

    Robbie robbie;
    robbie.platform.enable(TIME_STEP);

    auto pidConfig = config.pid;
    PID pid(-1, 1, pidConfig.p, pidConfig.i, pidConfig.d);

    WorldModel world;

    PathPlanning planner;
    MotionControl mc(pid, MotionControl::HEADING);
    mc.setSpeed(pidConfig.speed);

    Logging::Core->debug("Initialization complete");

    robbie.step(TIME_STEP);
    mc.setTarget(robbie.local.getHeading());
    Logging::Core->debug("Starting heading is {}", robbie.local.getHeading());

    planner.startUndock();

    while (robbie.step(TIME_STEP) != -1) {
        mc.update(&robbie.platform, &robbie.local);

        tel.send(&robbie);
        tel.send(&planner);
        tel.send(&mc);
    }

    return 0;
}
