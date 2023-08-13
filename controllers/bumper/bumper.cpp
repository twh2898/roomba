#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>
using namespace webots;

#include <iostream>
#include <memory>
#include <vector>
using namespace std;

#include "Localizer.hpp"
#include "MC.hpp"
#include "Planning.hpp"
#include "Roomba.hpp"
#include "Telemetry.hpp"

using namespace roomba;

#define SPEED 10
#define TIME_STEP 64

int main() {
    int movementCounter = 0;
    int leftSpeed, rightSpeed;

    Robot robot;
    Roomba roomba(&robot);
    roomba.enable(TIME_STEP);

    Telemetry tel(9870, "0.0.0.0");

    Localizer local;

    while (robot.step(TIME_STEP) != -1) {
        local.update(&roomba);

        if (roomba.bumper->getValue() > 0)
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

        roomba.leftMotor->setVelocity(leftSpeed);
        roomba.rightMotor->setVelocity(rightSpeed);

        tel.send(&roomba);
    }

    return 0;
}
