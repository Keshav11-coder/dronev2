#ifndef dronev2_pid_h
#define dronev2_pid_h

/** © 2024 Keshav Haripersad
 *  Basic PID logic class - pid.h
 */

// Requirements
#include <vector>

namespace dronev2
{
    struct pid
    {
    private:
        float Kp, Ki, Kd; // {Kp, Ki, Kd}
        unsigned long prevTime = 0, currentTime = 0;
        float deltaTime = 0, prevError = 0, integral = 0, controlOutput = 0;

    public:
        pid(float kp_, float ki_, float kd_);

        float run(float actual, float target);
        void adjust(float kp_, float ki_, float kd_);
        void reset();

        float getKp() const;
        float getKi() const;
        float getKd() const;
    };
}

#endif