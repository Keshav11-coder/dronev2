#ifndef dronev2_h
#define dronev2_h
#include <iostream>
#include <vector>
#include <functional>

#include "ESP32Servo.h"
#include "Mpu.h"

namespace dronev2
{
    struct motor
    {
    private:
        Servo esc;
        int esc_pin;
        int esc_min;
        int esc_max;
        bool armed = false;

    public:
        motor(int pin, int min, int max);

        struct failsafe_system
        {
        private:
            bool failsafe = false;

        public:
            bool engage();
            bool disengage();
            bool status();
        } failsafe;

        int arm();
        int stop();
        int restart();
        int max();
        int min();

        int write(int spd);
    };

    enum classification
    {
        fpv,
        cine
    };

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

    class task
    {
    private:
        unsigned long previous;
        std::function<void()> executable;

    public:
        int interval;
        int task_id = random(0, 1000);

        task(std::function<void()> callback, int ms) : previous(millis()), interval(ms), executable(callback) {}
        task(std::function<void()> callback, int ms, int _id) : previous(millis()), interval(ms), executable(callback), task_id(_id) {}

        int execute();
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

    class thread
    {
    private:
        std::vector<task> _tasks;

    public:
        template <typename... Tasks>
        thread(Tasks... tasks_)
        {
            (_tasks.push_back(tasks_), ...);
        }

        int add_task(task task_);
        int remove_task(int task_id);

        int update_task_interval(int task_id, int new_interval);

        int clear();
        void run();
    };

    class thread_ptr
    {
    private:
        std::vector<task *> _tasks;

    public:
        template <typename... Tasks>
        thread_ptr(Tasks... tasks_)
        {
            (_tasks.push_back(tasks_), ...);
        }

        int add_task(task *task_);
        int remove_task(int task_id);

        int update_task_interval(int task_id, int new_interval);

        int clear();
        void run();
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
