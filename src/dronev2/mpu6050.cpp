/** © 2024 Keshav Haripersad
 *  Proxy class for SJR mpu6050 class (sjr_mpu6050.h) - mpu6050.cpp
 */

#include "mpu6050.h"

namespace dronev2
{

    mpu6050::mpu6050() {}

    int mpu6050::begin(int sda, int scl)
    {
        return _imu.begin(sda, scl);
    }

    void mpu6050::calibrate()
    {
        _imu.calibrate();
    }

    mpu6050 &mpu6050::read()
    {
        _imu.read();
        yaw = _imu.yaw;
        pitch = _imu.pitch;
        roll = _imu.roll;
        ax = _imu.ax;
        ay = _imu.ay;
        az = _imu.az;
        gx = _imu.gx;
        gy = _imu.gy;
        gz = _imu.gz;
        ax_mps2 = _imu.ax_mps2;
        ay_mps2 = _imu.ay_mps2;
        az_mps2 = _imu.az_mps2;
        return *this;
    }

    // Setter and getter functions
    int mpu6050::updateGyroscopeOffsets(float *offsets)
    {
        return _imu.updateGyroscopeOffsets(offsets);
    }

    int mpu6050::updateAccelerometerCalibrations(float *calibrations)
    {
        return _imu.updateAccelerometerCalibrations(calibrations);
    }

    int mpu6050::updateGyroScale(float scale)
    {
        return _imu.updateGyroScale(scale);
    }

    int mpu6050::changeAddress(int address)
    {
        return _imu.changeAddress(address);
    }

    int mpu6050::updateMahonyKp(float Kp_)
    {
        return _imu.updateMahonyKp(Kp_);
    }

    int mpu6050::updateMahonyKi(float Ki_)
    {
        return _imu.updateMahonyKi(Ki_);
    }

    int mpu6050::changeCalibrationFlag(int calibrateGyro_)
    {
        return _imu.changeCalibrationFlag(calibrateGyro_);
    }

    int mpu6050::updateAccelSensitivity(float sensitivity)
    {
        return _imu.updateAccelSensitivity(sensitivity);
    }

    int mpu6050::changeGravity(float gravity_)
    {
        return _imu.changeGravity(gravity_);
    }
}