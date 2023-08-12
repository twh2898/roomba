#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
using namespace webots;

#include <iostream>
#include <memory>
#include <vector>
using namespace std;

#define SPEED 3
#define TIME_STEP 64

int main() {
    int movementCounter = 0;
    int leftSpeed, rightSpeed;

    Robot robot;

    auto leftMotor = robot.getMotor("left wheel motor");
    auto rightMotor = robot.getMotor("right wheel motor");

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    while (robot.step(TIME_STEP) != -1) {
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

        leftMotor->setVelocity(leftSpeed);
        rightMotor->setVelocity(rightSpeed);
    }

    return 0;
}
