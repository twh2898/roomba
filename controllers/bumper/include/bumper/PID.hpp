#pragma once

/* https://gist.github.com/bradley219/5373998
 * Copyright 2019 Bradley J. Snyder <snyder.bradleyj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Telemetry.hpp"

namespace roomba {
    class PID : public TelemetrySender {
        double max;
        double min;
        double Kp;
        double Ki;
        double Kd;
        double preError;
        double integral;

    public:
        /**
         * @param min minimum value of manipulated variable
         * @param max maximum value of manipulated variable
         * @param Kp proportional gain
         * @param Ki Integral gain
         * @param Kd derivative gain
         */
        PID(double min, double max, double Kp, double Ki, double Kd);

        /**
         * Reset the previousError and integral to 0.
         */
        void reset();

        /**
         * Returns the manipulated variable given a setPoint and current
         * process value.
         *
         * @param dt loop interval time
         * @param setPoint target value
         * @param processValue current value
         *
         * @throw std::runtime_error if dt <= 0.0
         */
        double calculate(double dt, double setPoint, double processValue);

        json getTelemetry() const override;
    };
}