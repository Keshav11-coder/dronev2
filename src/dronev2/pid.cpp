/** © 2024 Keshav Haripersad
 *  Basic PID logic class - pid.cpp
 */

// Requirements
#include <vector>
#include <Arduino.h>

// Header
#include "pid.h"

namespace dronev2
{
    pid::pid(float kp_, float ki_, float kd_)
    {
        Kp = kp_;
        Ki = ki_;
        Kd = kd_;
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
        float proportional = Kp * error;

        integral += error * deltaTime;
        if (integral > 100)
            integral = 100;
        if (integral < -100)
            integral = -100;

        float derivative = (error - prevError) / deltaTime;
        controlOutput = proportional + (Ki * integral) + (Kd * derivative);
        prevError = error;

        return controlOutput;
    }

    void pid::adjust(float kp_, float ki_, float kd_)
    {
        if (kp_ != 0 && ki_ != 0 && kd_ != 0)
        {
            Kp = kp_;
            Ki = ki_;
            Kd = kd_;
        }
    }

    void pid::reset()
    {
        prevError = 0;
        integral = 0;
        prevTime = millis();
    }

    float pid::getKp() const
    {
        return Kp;
    }

    float pid::getKi() const
    {
        return Ki;
    }

    float pid::getKd() const
    {
        return Kd;
    }
}