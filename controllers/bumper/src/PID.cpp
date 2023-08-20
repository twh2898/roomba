#include "bumper/PID.hpp"

#include <exception>

namespace roomba {
    using std::runtime_error;

    PID::PID(double min, double max, double Kp, double Ki, double Kd)
        : min(min), max(max), Kp(Kp), Ki(Ki), Kd(Kd), preError(0), integral(0) {}

    void PID::reset() {
        preError = 0;
        integral = 0;
    }

    double PID::calculate(double dt, double setPoint, double processValue) {
        if (dt <= 0.0)
            throw runtime_error("dt must be greater than 0");

        double error = setPoint - processValue;

        // Proportional term
        double Pout = Kp * error;

        // Integral term
        integral += error * dt;
        double Iout = Ki * integral;

        // Derivative term
        double derivative = (error - preError) / dt;
        double Dout = Kd * derivative;

        double output = Pout + Iout + Dout;

        if (output > max)
            output = max;
        else if (output < min)
            output = min;

        preError = error;

        return output;
    }

    json PID::getTelemetry() const {
        return json {
            {"integral", integral},
            {"preError", preError},
        };
    }
}
