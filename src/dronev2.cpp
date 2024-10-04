#include "dronev2.h"

dronev2::motor::motor(int pin, int min, int max)
    : esc_pin(pin), esc_min(min), esc_max(max)
{
    esc.attach(pin);
}

bool dronev2::motor::failsafe_system::engage()
{
    failsafe = true;
    return failsafe;
}

bool dronev2::motor::failsafe_system::disengage()
{
    failsafe = false;
    return failsafe;
}

bool dronev2::motor::failsafe_system::status()
{
    return failsafe;
}

int dronev2::motor::arm()
{
    if (!failsafe.status())
    {
        esc.writeMicroseconds(esc_min);
        armed = true;
        return 0;
    }
    return 1;
}

int dronev2::motor::stop()
{
    if (armed && !failsafe.status())
    {
        esc.writeMicroseconds(esc_min);
        failsafe.engage();
        return 0;
    }
    return 1;
}

int dronev2::motor::restart()
{
    if (armed && failsafe.status())
    {
        esc.writeMicroseconds(esc_min);
        failsafe.disengage();
        return 0;
    }
    return 1;
}

int dronev2::motor::max()
{
    return esc_max;
}

int dronev2::motor::min()
{
    return esc_min;
}

int dronev2::motor::write(int spd)
{
    if (armed && !failsafe.status())
    {
        if (spd >= esc_min && spd <= esc_max)
        {
            esc.writeMicroseconds(spd);
            return 0;
        }
        else
        {
            return 1;
        }
    }
    return 1;
}

/** PID systems
 *  - base functions for pid implementations
 */

dronev2::pid::pid(std::vector<float> initial_coefficients)
{
    coefficients = initial_coefficients;
    prevTime = millis();
}

float dronev2::pid::run(float actual, float target)
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

void dronev2::pid::adjust(std::vector<float> new_coefficients)
{
    if (new_coefficients.size() == 3)
        coefficients = new_coefficients;
}

void dronev2::pid::reset()
{
    prevError = 0;
    integral = 0;
    prevTime = millis();
}

std::vector<float> dronev2::pid::get_coefficients() const
{
    return coefficients;
}

int dronev2::task::execute()
{
    unsigned long currentMillis = millis();
    if (currentMillis - previous >= interval)
    {
        previous = currentMillis;
        executable();
        return 0;
    }
    return -1;
}

int dronev2::thread::add_task(task task_)
{
    _tasks.push_back(task_);
    return 0;
}

int dronev2::thread::remove_task(int task_id)
{
    for (int i = 0; i < _tasks.size(); i++)
    {
        if (_tasks[i].task_id == task_id)
        {
            _tasks.erase(_tasks.begin() + i);
            return 0;
        }
    }
    return 1;
}

int dronev2::thread::update_task_interval(int task_id, int new_interval)
{
    for (task task_ : _tasks)
    {
        if (task_.task_id == task_id)
        {
            task_.interval = new_interval;
        }
    }
    return 0;
}

int dronev2::thread::clear()
{
    _tasks.clear();
    return 0;
}

void dronev2::thread::run()
{
    for (int i = 0; i < _tasks.size(); i++)
    {
        _tasks[i].execute();
    }
}

int dronev2::thread_ptr::add_task(task *task_)
{
    _tasks.push_back(task_);
    return 0;
}

int dronev2::thread_ptr::remove_task(int task_id)
{
    for (int i = 0; i < _tasks.size(); i++)
    {
        if (_tasks[i]->task_id == task_id)
        {
            _tasks.erase(_tasks.begin() + i);
            return 0;
        }
    }
    return 1;
}

int dronev2::thread_ptr::update_task_interval(int task_id, int new_interval)
{
    for (task *task_ : _tasks)
    {
        if (task_->task_id == task_id)
        {
            task_->interval = new_interval;
        }
    }
    return 0;
}

int dronev2::thread_ptr::clear()
{
    for (task *ptr : _tasks)
    {
        delete ptr;
    }
    _tasks.clear();
    return 0;
}

void dronev2::thread_ptr::run()
{
    for (int i = 0; i < _tasks.size(); i++)
    {
        _tasks[i]->execute();
    }
}

dronev2::drone &dronev2::drone::stabilization(std::function<void(imu_data_packaged)> stabilization_exec, int stabilization_rate)
{
    _thread.remove_task(574);
    _thread.add_task(new task([stabilization_exec, this]()
                              {
                                        imu_data_packaged imu_data_;
                                        imu_data_.yaw = this->mpu.yaw;
                                        imu_data_.pitch = this->mpu.pitch;
                                        imu_data_.roll = this->mpu.roll;
                                        imu_data_.gx = this->mpu.gx;
                                        imu_data_.gy = this->mpu.gy;
                                        imu_data_.gz = this->mpu.gz;
                                        stabilization_exec(imu_data_); }, stabilization_rate, 574));
    return *this;
}

dronev2::drone &dronev2::drone::imu(int sda, int scl)
{
    mpu.configure(sda, scl);
    return *this;
}

int dronev2::drone::add_task(task *task_)
{
    _thread.add_task(task_);
    return 0;
}

int dronev2::drone::update_task_interval(int task_id, int new_interval)
{
    _thread.update_task_interval(task_id, new_interval);
    return 0;
}

int dronev2::drone::remove_task(int task_id)
{
    _thread.remove_task(task_id);
    return 0;
}

int dronev2::drone::clear_tasks()
{
    _thread.clear();
    return 0;
}

int dronev2::drone::persist_tasks()
{
    _thread.run();
    return 0;
}

int dronev2::drone::arm_all()
{
    for (motor *motor_ : _motors)
    {
        motor_->arm();
    }
    return 0;
}

int dronev2::drone::write_all(int spd)
{
    for (motor *motor_ : _motors)
    {
        motor_->write(spd);
    }
    return 0;
}