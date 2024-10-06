// Requirements
#include <vector>
#include <Arduino.h>

// Header pid.h
#include "pid.h"

namespace dronev2
{
    pid::pid(std::vector<float> initial_coefficients)
    {
        coefficients = initial_coefficients;
        prevTime = millis();
    }

    float pid::run(float actual, float target)
    {
        currentTime = millis();
        deltaTime = (currentTime - prevTime) / 1000.0;
        if (deltaTime <= 0)
        {
            deltaTime = 0.001;
        }
        prevTime = currentTime;

        float error = target - actual;
        float proportional = coefficients[0] * error;

        integral += error * deltaTime;
        if (integral > 100)
            integral = 100;
        if (integral < -100)
            integral = -100;

        float derivative = (error - prevError) / deltaTime;
        controlOutput = proportional + (coefficients[1] * integral) + (coefficients[2] * derivative);
        prevError = error;

        return controlOutput;
    }

    void pid::adjust(std::vector<float> new_coefficients)
    {
        if (new_coefficients.size() == 3)
            coefficients = new_coefficients;
    }

    void pid::reset()
    {
        prevError = 0;
        integral = 0;
        prevTime = millis();
    }

    std::vector<float> pid::get_coefficients() const
    {
        return coefficients;
    }
}