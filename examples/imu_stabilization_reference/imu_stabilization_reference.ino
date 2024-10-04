#include <dronev2.h>
#include <Wire.h>

/**
 * This example just explains how you can use the imu values.
 *  - Showcasing imu usage
 *  - Demonstrating a basic stabilization system
 *
 * Note: When reading this example's documentation, the writer presumes you have a decent amount of knowledge about the drone's API.
 */

// Define your drone instance
dronev2::drone quadcopter;

// Define dronev2 namespace included pid instances
dronev2::pid yaw_pid({3.55, 0.005, 0.0001});
dronev2::pid pitch_pid({3.55, 0.005, 0.0001});
dronev2::pid roll_pid({3.55, 0.005, 0.0001});

void setup()
{
    Serial.begin(115200);

    randomSeed(analogRead(0));

    // Initialization.
    // The barebone of the drone's initialization chain looks like this.
    // At it's most basic, we only have motor initialization and imu initialization.
    quadcopter

        // Motor initiaization is pretty simple. All you have to do is create new pointer objects and specify (pin, speed_min, speed_max) with the speeds being in microseconds.
        // The amount of new motor objects can be anything you like, from 4 to 8 to 16. As long as they are properly managed by you in the future (see motor speed writing at the bottom)
        .motors(
            new dronev2::motor(13, 1000, 2000),
            new dronev2::motor(22, 1000, 2000),
            new dronev2::motor(15, 1000, 2000),
            new dronev2::motor(26, 1000, 2000))

        // Stabilization.
        // Getting the imu data via the stabilization chain

        // Stabilization is a crucial part of the drone, so I decided to place a blockchain that compiles the lambda code into a task that gets added to the default thread.
        // The stabilization lambda offers the imu data as packaged (in a struct) as an argument to your function. You can use other local variables too by capturing them.

        // We capture our pid systems so we can use them. We also grab the imu_data_packaged from the chain.
        // If you look at the end of those argument and capture definitions, yo uwill see a *mutable* keyword.
        // This keyword (in simple terms) allows the labmda to change over time and adjust, which is crucial for a stabilization system that has to adjust to many different scenarios.
        // This can of course be extended further with more advanced systems in the future.
        .stabilization([yaw_pid, pitch_pid, roll_pid](dronev2::imu_data_packaged imu_data) mutable {
            float _pitch_;
            float _roll_;
            float _yaw_;

            _pitch_ = -pitch_pid.run(imu_data.pitch, 0);
            _roll_ = roll_pid.run(imu_data.roll, 0);
            _yaw_ = -yaw_pid.run(imu_data.yaw, 0);

            Serial.print(constrain((1200 - _yaw_ + _pitch_ - _roll_), 1000, 2000), 0);
            Serial.print(constrain((1200 + _yaw_ + _pitch_ + _roll_), 1000, 2000), 0);
            Serial.print(constrain((1200 + _yaw_ - _pitch_ - _roll_), 1000, 2000), 0);
            Serial.println(constrain((1200 - _yaw_ - _pitch_ + _roll_), 1000, 2000), 0);
        }, 5) // Specify a small stabilization rate to not overflow, but also not be too slow.

        // Imu initialization is fairly simple as well. Though we don't yet support custom TwoWire objects, you can specify your sda and scl pins respectively for now.
        // This way you still have some kind of customization, and if you wish to go more advanced you can use tasks (see other examples) to interface with the imu in a different way.
        .imu(19, 23);


    /** MPU (imu) API
      * - Getting the imu data via the mpu member variable
    */

    // This is fairly short. The mpu member variable is public and allows you to grab the euler angles (yaw, pitch & roll) as well as the raw gyro values (gx, gy & gz) directly.
    // quadcopter.mpu.yaw
    // quadcopter.mpu.pitch
    // quadcopter.mpu.roll
    // quadcopter.mpu.gx
    // quadcopter.mpu.gy
    // quadcopter.mpu.gz

    // Notes:
    //  - The Mpu.h (the base handler for the imu) looks pretty mess at the time of writing and will be structurally updated in the future. Look out for updates!
    //  - Make sure to actually run the imu so you can get accurate values. This feature was originally added for possibly periodic imu data.
}

void loop()
{
    quadcopter.mpu.run(); // run the mpu
    quadcopter.persist_tasks();
}