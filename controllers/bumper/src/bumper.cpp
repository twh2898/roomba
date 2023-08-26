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

#include "bumper/Localizer.hpp"
#include "bumper/MotionControl.hpp"
#include "bumper/PID.hpp"
#include "bumper/Planning.hpp"
#include "bumper/Roomba.hpp"
#include "bumper/Telemetry.hpp"
#include "bumper/log.hpp"

using namespace roomba;

#define TIME_STEP 64

namespace roomba::Logging {
    Logger::Ptr Core;
}

int main() {
    Logging::Core = make_shared<Logging::Logger>("Roomba");
    Logging::Core->setLevel(Logging::Logger::Debug);
    Logging::Core->debug("Logging enabled");

    fstream f("config.json");
    json config = json::parse(f);

    {
        string c = config.dump();
        Logging::Core->debug("Config: {}", c);
    }

    int movementCounter = 0;
    int leftSpeed, rightSpeed;

    Robot robot;
    Roomba roomba(&robot);
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
