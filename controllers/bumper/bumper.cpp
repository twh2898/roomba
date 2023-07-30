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

#include <webots/accelerometer.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>

#include <iostream>
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
    WbDeviceTag bumper, accel;
    WbDeviceTag left_motor, right_motor;
    int movement_counter = 0;
    int left_speed, right_speed;

    wb_robot_init();

    /* get a handle the the bumper and activate it. */
    bumper = wb_robot_get_device("bumper");
    wb_touch_sensor_enable(bumper, TIME_STEP);

    accel = wb_robot_get_device("accel");
    wb_accelerometer_enable(accel, TIME_STEP);

    /* get a handler to the motors and set target position to infinity (speed
     * control) */
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);

    UDPClient client(9870, "0.0.0.0");

    /* control loop */
    while (wb_robot_step(TIME_STEP) != -1) {
        /*
         * When the touch sensor has detected something we begin the avoidance
         * movement.
         */
        if (wb_touch_sensor_get_value(bumper) > 0)
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

        auto * comp = wb_accelerometer_get_values(accel);
        cout << "X " << comp[0] << " Y " << comp[1] << " Z " << comp[2] << endl;


        // Using initializer lists
        json ex3 = {
            {"x", comp[0]},
            {"y", comp[1]},
            {"z", comp[2]},
            {"time", wb_robot_get_time()},
        };
        client.send_message(ex3.dump());

        // if (comp[0] > 0.2) {
        //     movement_counter = 15;
        // }

        /* Set the motor speeds. */
        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);
    }

    wb_robot_cleanup();

    return 0;
}
