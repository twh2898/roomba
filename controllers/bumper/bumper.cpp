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

#include "Loc.hpp"
#include "MC.hpp"
#include "Planning.hpp"
#include "Roomba.hpp"
#include "json.hpp"
#include "simple_cpp_sockets.h"

using json = nlohmann::json;

using namespace roomba;

#define SPEED 10
#define TIME_STEP 64

int main() {
    int movementCounter = 0;
    int leftSpeed, rightSpeed;

    Robot robot;
    Roomba roomba(&robot);
    roomba.enable(TIME_STEP);

    UDPClient client(9870, "0.0.0.0");

    Localizer local(roomba.accel, roomba.leftMotor, roomba.rightMotor);

    while (robot.step(TIME_STEP) != -1) {
        local.update();

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

        auto * comp = roomba.accel->getValues();

        json accelData = {
            {"accel",
             {
                 {"x", comp[0]},
                 {"y", comp[1]},
                 {"z", comp[2]},
             }},
            {"left",
             {
                 {"velocity", roomba.leftMotor->getVelocity()},
             }},
            {"right",
             {
                 {"velocity", roomba.rightMotor->getVelocity()},
             }},
            {"local",
             {
                 {"velocity",
                  {
                      {"x", local.velX},
                      {"y", local.velY},
                  }},
                 {"position",
                  {
                      {"x", local.posX},
                      {"y", local.posY},
                  }},
             }},
            {"time", robot.getTime()},
        };
        client.send_message(accelData.dump());

        roomba.leftMotor->setVelocity(leftSpeed);
        roomba.rightMotor->setVelocity(rightSpeed);
    }

    return 0;
}
