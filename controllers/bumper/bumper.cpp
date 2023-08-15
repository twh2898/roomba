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
    fstream f("pid.json");
    json pidConfig = json::parse(f);

    cout << "PID config " << pidConfig.dump() << endl;

    int movementCounter = 0;
    int leftSpeed, rightSpeed;

    Robot robot;
    Roomba roomba(&robot);
    roomba.enable(TIME_STEP);

    Telemetry tel(9870, "0.0.0.0");

    Localizer local;
    MotionControl mc;

    auto mode = pidConfig["mode"];

    // auto twistConfig = pidConfig[mode];
    // const double speed = twistConfig["speed"];
    // PID pid(1, -1, twistConfig["p"], twistConfig["i"], twistConfig["d"]);

    double target = pidConfig["target"];
    mc.setMode(MotionControl::HEADING);
    mc.setTarget(target);
    mc.setDrive(pidConfig["drive"]);

    while (robot.step(TIME_STEP) != -1) {
        local.update(&roomba);
        mc.update(&roomba, &local);

        tel.send(&roomba);
        tel.send(&local);
        tel.send(&mc);
    }

    return 0;
}
