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

#include "robbie/Localizer.hpp"
#include "robbie/MotionControl.hpp"
#include "robbie/PID.hpp"
#include "robbie/Planning.hpp"
#include "robbie/Platform.hpp"
#include "robbie/Telemetry.hpp"
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

    fstream f("config.json");
    json config = json::parse(f);

    {
        string c = config.dump();
        Logging::Core->debug("Config: {}", c);
    }

    int movementCounter = 0;
    int leftSpeed, rightSpeed;

    Robot robot;
    Platform roomba(&robot);
    roomba.enable(TIME_STEP);

    Telemetry tel(9870, "0.0.0.0");

    bool tuneMode = config["tuneMode"];

    auto mode = config["pid"]["mode"];
    auto pidConfig = config["pid"][mode];
    PID pid(-1, 1, pidConfig["p"], pidConfig["i"], pidConfig["d"]);

    Localizer local;
    PathPlanning planner;
    MotionControl mc(pid, MotionControl::HEADING);
    mc.setSpeed(pidConfig["speed"]);

    vector<XY> path;
    for (auto xy : config["path"]) {
        path.emplace_back(xy["x"], xy["y"]);
    }

    auto target = config["target"];

    if (tuneMode) {
        mc.setTarget(target["heading"]);
        mc.setDrive(config["drive"]);
    }
    else {
        // planner.setZoneSize(target["size"]);
        // planner.setPath(path);
    }

    mc.setTarget(M_PI);

    Logging::Core->debug("Initialization complete");

    robot.step(TIME_STEP);
    local.update(&roomba);
    mc.setTarget(local.heading);
    Logging::Core->debug("Starting heading is {}", local.heading);

    planner.startUndock();

    while (robot.step(TIME_STEP) != -1) {
        local.update(&roomba);
        if (!tuneMode)
            planner.update(&roomba, &local, &mc);
        mc.update(&roomba, &local);

        tel.send(&roomba);
        tel.send(&local);
        tel.send(&planner);
        tel.send(&mc);
    }

    return 0;
}
