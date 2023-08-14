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
#include "MC.hpp"
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

    auto mode = pidConfig["mode"];

    auto twistConfig = pidConfig[mode];
    const double speed = twistConfig["speed"];
    PID pid(1, -1, twistConfig["p"], twistConfig["i"], twistConfig["d"]);

    double target = pidConfig["target"];

    while (robot.step(TIME_STEP) != -1) {
        local.update(&roomba);

        double yaw = roomba.imu->getRollPitchYaw()[2];

        double e = target - roomba.imu->getRollPitchYaw()[2];
        if (e < -M_PI)
            e = -(e - M_PI);
        else if (e > M_PI)
            e = -(e - M_PI);

        tel.sendCustom({{"e", -e}});

        double twist = pid.calculate(TIME_STEP, 0, -e);
        tel.sendCustom({
            {"target", target},
            {"yaw", yaw},
            {"twist", twist},
        });

        leftSpeed = -speed * twist;
        rightSpeed = speed * twist;

        // if (roomba.bumper->getValue() > 0)
        //     movementCounter = 15;

        // if (movementCounter == 0) {
        //     leftSpeed = SPEED;
        //     rightSpeed = SPEED;
        // }
        // else if (movementCounter >= 7) {
        //     leftSpeed = -SPEED;
        //     rightSpeed = -SPEED;
        //     movementCounter--;
        // }
        // else {
        //     leftSpeed = -SPEED / 2;
        //     rightSpeed = SPEED;
        //     movementCounter--;
        // }

        roomba.leftMotor->setVelocity(leftSpeed);
        roomba.rightMotor->setVelocity(rightSpeed);

        tel.send(&roomba);
    }

    return 0;
}
