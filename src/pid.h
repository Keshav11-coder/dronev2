#ifndef dronev2_pid_h
#define dronev2_pid_h

// Requirements
#include <vector>

namespace dronev2
{
    struct pid
    {
    private:
        std::vector<float> coefficients = {1, 1, 1}; // {Kp, Ki, Kd}
        unsigned long prevTime = 0, currentTime = 0;
        float deltaTime = 0, prevError = 0, integral = 0, controlOutput = 0;

    public:
        pid(std::vector<float> initial_coefficients);

        float run(float actual, float target);
        void adjust(std::vector<float> new_coefficients);
        void reset();

        std::vector<float> get_coefficients() const;
    };
}

#endif