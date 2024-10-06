#ifndef dronev2_h
#define dronev2_h
#include <iostream>
#include <vector>
#include <functional>

// Logical headers the drone needs as a fallback, but implementation shows no requirements.
#include "Mpu.h"

// dronev2 requires these libraries for its base.
#include "motor.h"
#include "tasker.h"

namespace dronev2
{
    enum classification
    {
        fpv,
        cine
    };

    struct imu_data_packaged
    {
        float yaw = 0;
        float pitch = 0;
        float roll = 0;
        float gx = 0;
        float gy = 0;
        float gz = 0;
    };

    class drone
    {
    private:
        std::vector<motor *> _motors = {
            new motor(15, 1000, 2000),
            new motor(16, 1000, 2000),
            new motor(4, 1000, 2000),
            new motor(5, 1000, 2000)};
        classification _class = cine;

    public:
        MPU mpu;
        thread_ptr _thread;
        drone() : _thread()
        {
        }

        template <typename... Motors>
        drone &motors(Motors... motors)
        {
            // Motor initializer. Make sure to clear the _motors array first.
            // We do this to avoid stacking .motor initializers.
            // This way which .motor chain end comes last declares the array.
            for (motor *ptr : _motors)
            {
                delete ptr;
            }
            _motors.clear();
            (_motors.push_back(motors), ...);
            return *this;
        }

        drone &stabilization(std::function<void(imu_data_packaged)> stabilization_exec, int stabilization_rate);
        drone &imu(int sda, int scl);

        int add_task(task *task_);
        int update_task_interval(int task_id, int new_interval);
        int remove_task(int task_id);
        int clear_tasks();
        int persist_tasks();
        int arm_all();
        int write_all(int spd);

        template <typename... MotorSpeeds>
        int write_all(MotorSpeeds... motor_speeds_)
        {
            std::vector<float> motor_speeds = {motor_speeds_...};
            if (motor_speeds.size() == _motors.size())
            {
                for (int i = 0; i < _motors.size(); i++)
                {
                    _motors[i]->write(motor_speeds[i]);
                }
                return 0;
            }
            else
            {
                return 1;
            }
        }
    };
}

#endif
