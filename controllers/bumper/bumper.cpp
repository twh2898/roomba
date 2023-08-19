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

#include "Localizer.hpp"
#include "MotionControl.hpp"
#include "PID.hpp"
#include "Planning.hpp"
#include "Roomba.hpp"
#include "Telemetry.hpp"

using namespace roomba;

#define TIME_STEP 64

int main() {
    fstream f("config.json");
    json config = json::parse(f);

    cout << "Config " << config.dump() << endl;

    int movementCounter = 0;
    int leftSpeed, rightSpeed;

    Robot robot;
    Roomba roomba(&robot);
    roomba.enable(TIME_STEP);

    Telemetry tel(9870, "0.0.0.0");

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
    mc.setTarget(target["heading"]);
    // mc.setDrive(config["drive"]);

    // planner.setZoneSize(target["size"]);
    // planner.setTarget(target["x"], target["y"]);
    // planner.setPath(path);

    while (robot.step(TIME_STEP) != -1) {
        local.update(&roomba);
        // planner.update(&roomba, &local, &mc);
        mc.update(&roomba, &local);

        tel.send(&roomba);
        tel.send(&local);
        tel.send(&planner);
        tel.send(&mc);
    }

    return 0;
}
