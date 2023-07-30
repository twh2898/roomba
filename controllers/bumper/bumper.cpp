#include <webots/Accelerometer.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>
using namespace webots;

#include <iostream>
#include <memory>
#include <vector>
using namespace std;

#include "Loc.hpp"
#include "MC.hpp"
#include "Planning.hpp"
#include "json.hpp"
#include "simple_cpp_sockets.h"

using json = nlohmann::json;

#define SPEED 3
#define TIME_STEP 64

int main() {
    int movementCounter = 0;
    int leftSpeed, rightSpeed;

    auto robot = make_shared<Robot>();

    auto bumper = robot->getTouchSensor("bumper");
    bumper->enable(TIME_STEP);

    auto accel = robot->getAccelerometer("accel");
    accel->enable(TIME_STEP);

    auto leftMotor = robot->getMotor("left wheel motor");
    auto rightMotor = robot->getMotor("right wheel motor");

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    UDPClient client(9870, "0.0.0.0");

    while (robot->step(TIME_STEP) != -1) {
        if (bumper->getValue() > 0)
            movementCounter = 15;

        if (movementCounter == 0) {
            leftSpeed = SPEED;
            rightSpeed = SPEED;
        }
        else if (movementCounter >= 7) {
            leftSpeed = -SPEED;
            rightSpeed = -SPEED;
            movementCounter--;
        }
        else {
            leftSpeed = -SPEED / 2;
            rightSpeed = SPEED;
            movementCounter--;
        }

        auto * comp = accel->getValues();
        cout << "X " << comp[0] << " Y " << comp[1] << " Z " << comp[2] << endl;

        json accelData = {
            {"x", comp[0]},
            {"y", comp[1]},
            {"z", comp[2]},
            {"time", robot->getTime()},
        };
        client.send_message(accelData.dump());

        // if (comp[0] > 0.2) {
        //     movementCounter = 15;
        // }

        leftMotor->setVelocity(leftSpeed);
        rightMotor->setVelocity(rightSpeed);
    }

    return 0;
}
