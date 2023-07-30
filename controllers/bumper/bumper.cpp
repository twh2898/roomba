/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  An example of use of a bumper touch sensor device.
 */


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
    int movement_counter = 0;
    int left_speed, right_speed;

    auto robot = make_shared<Robot>();

    auto bumper = robot->getTouchSensor("bumper");
    bumper->enable(TIME_STEP);

    auto accel = robot->getAccelerometer("accel");
    accel->enable(TIME_STEP);

    auto left_motor = robot->getMotor("left wheel motor");
    auto right_motor = robot->getMotor("right wheel motor");

    left_motor->setPosition(INFINITY);
    right_motor->setPosition(INFINITY);

    left_motor->setVelocity(0.0);
    right_motor->setVelocity(0.0);

    UDPClient client(9870, "0.0.0.0");

    /* control loop */
    while (robot->step(TIME_STEP) != -1) {
        if (bumper->getValue() > 0)
            movement_counter = 15;

        /*
         * We use the movement_counter to manage the movements of the robot.
         * When the value is 0 we move straight, then when there is another
         * value this means that we are avoiding an obstacle. For avoiding we
         * first move backward for some cycles and then we turn on ourself.
         */
        if (movement_counter == 0) {
            left_speed = SPEED;
            right_speed = SPEED;
        }
        else if (movement_counter >= 7) {
            left_speed = -SPEED;
            right_speed = -SPEED;
            movement_counter--;
        }
        else {
            left_speed = -SPEED / 2;
            right_speed = SPEED;
            movement_counter--;
        }

        auto * comp = accel->getValues();
        cout << "X " << comp[0] << " Y " << comp[1] << " Z " << comp[2] << endl;


        // Using initializer lists
        json ex3 = {
            {"x", comp[0]},
            {"y", comp[1]},
            {"z", comp[2]},
            {"time", robot->getTime()},
        };
        client.send_message(ex3.dump());

        // if (comp[0] > 0.2) {
        //     movement_counter = 15;
        // }

        /* Set the motor speeds. */
        left_motor->setVelocity(left_speed);
        right_motor->setVelocity(right_speed);
    }

    return 0;
}
