#ifndef dronev2_mpu6050_h
#define dronev2_mpu6050_h

/** © 2024 Keshav Haripersad
 *  Proxy class header for SJR mpu6050 class (sjr_mpu6050.h) - mpu6050.h
 */

#include "tools/sjr_mpu6050.h"

namespace dronev2
{

    class mpu6050
    {
    private:
        sjr_mpu_6050 _imu;

    public:
        float yaw, pitch, roll;
        int16_t ax, ay, az;
        int16_t gx, gy, gz;

        float ax_mps2, ay_mps2, az_mps2;

        mpu6050();

        int begin(int sda, int scl);
        void calibrate();
        mpu6050 &read();

        // Setter and getter functions
        int updateGyroscopeOffsets(float *offsets);
        int updateAccelerometerCalibrations(float *calibrations);
        int updateGyroScale(float scale);
        int changeAddress(int address);
        int updateMahonyKp(float Kp_);
        int updateMahonyKi(float Ki_);
        int changeCalibrationFlag(int calibrateGyro_);
        int updateAccelSensitivity(float sensitivity);
        int changeGravity(float gravity_);
    };
}

#endif