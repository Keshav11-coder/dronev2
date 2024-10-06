/** Base header - dronev2.h
 *  Includes base logic. Check out other modules mentioned.
 *
 *  dronev2, 2024 licensed under the MIT license.
 *  author: Keshav Haripersad
 *  github: https://github.com/Keshav11-coder/dronev2
 *
 */

// Header
#include "dronev2.h"

// Requirements (needed for base setup)
#include "motor.h"
#include "tasker.h"

// Logic
namespace dronev2
{
    drone &dronev2::drone::stabilization(std::function<void(imu_data_packaged)> stabilization_exec, int stabilization_rate)
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

    drone &dronev2::drone::imu(int sda, int scl)
    {
        mpu.configure(sda, scl);
        return *this;
    }

    int drone::add_task(task *task_)
    {
        _thread.add_task(task_);
        return 0;
    }

    int drone::update_task_interval(int task_id, int new_interval)
    {
        _thread.update_task_interval(task_id, new_interval);
        return 0;
    }

    int drone::remove_task(int task_id)
    {
        _thread.remove_task(task_id);
        return 0;
    }

    int drone::clear_tasks()
    {
        _thread.clear();
        return 0;
    }

    int drone::persist_tasks()
    {
        _thread.run();
        return 0;
    }

    int drone::arm_all()
    {
        for (motor *motor_ : _motors)
        {
            motor_->arm();
        }
        return 0;
    }

    int drone::write_all(int spd)
    {
        for (motor *motor_ : _motors)
        {
            motor_->write(spd);
        }
        return 0;
    }
}