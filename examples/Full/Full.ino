/** © 2025 Keshav Haripersad
 *  dronev2 - Full example
 *  
 *  This example shows a way to use this library. Including all aspects from other examples and integrating
 *  advanced systems and operations. Everything is explained on an advanced level.
 * 
 *  Licensed under the MIT license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/dronev2
 */

#include <dronev2.h> // Main API (major things such as `Profile`, `Drone`, `drone_event_t` can be found here)
#include <dronev2/tasker.h> // Built-in tasking system (also includes `thread` for combining tasks to prevent clutter)
#include <dronev2/mpu6050.h> // Built-in imu class
#include <dronev2/bmp280.h> // Built-in barometer class
#include <dronev2/ESC.h> // Built-in ESC class

// Normalize namespace `dronev2` to prevent messy, unreadable code
using namespace dronev2;

// Drone configuration (profile)
Profile configuration = Profile()
  // [A] Describes distribution of forces along motors (columns define motors, rows define components)
  // Axis components (tau_x, tau_y, tau_z) include geometry descriptions
  .setAllocationMatrix({
    {1.1, 1.1, 1.1, 1.1}, // Motor contribution component
    {0.2 * 1.1 / sqrt(2), -0.2 * 1.1 / sqrt(2), -0.2 * 1.1 / sqrt(2), 0.2 * 1.1 / sqrt(2)}, // Pitch torque component (tau_x)
    {-0.2 * 1.1 / sqrt(2), -0.2 * 1.1 / sqrt(2), 0.2 * 1.1 / sqrt(2), 0.2 * 1.1 / sqrt(2)}, // Roll torque component (tau_y)
    {-0.55, 0.55, -0.55, 0.55},  // Yaw torque component (tau_z)
    {0.2, -0.2, 0.2, -0.2},  // Additional component, normalized for stability.
    {0.2, 0.2, -0.2, -0.2}   // Additional component, normalized for stability.
  })

  // [I] Describes the resistance of rotating forces along each frame axis.
  .setInertiaMatrix({
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0}
  })

  // Typical static parameters
  .setMass(0.8) // Drone mass
  .setPropellerDiameter(0.127) // Propeller diameter
  .setPropellerPitch(0.1143) // Propeller pitch
  .setArmLength(0.2) // Arm length
  .setMotorKvRating(2300) // Motor Kv rating
  .setOperatingVoltage(11.1); // Operating voltage

// Drone constructor
// 1. Define sensors that will be used in template (<...>)
// 2. Provide a pointer to a configuration (profile)
Drone<mpu6050, bmp280> drone(&configuration);

// [M] Motor thrusts vector: 4 motors
matrix motorThrusts(4, 1, 1);

// [F] Wrench (desired forces) for all 6 components: 
// rows 1—3 (acceleration) Tx, Ty, Tz
// rows 4—6 (rotation) tau_x, tau_y, tau_z
matrix wrench(6, 1, 1);

// Compute desired orientation (rotational forces tau_x, tau_y, tau_z) every 5 ms
// 1. Read the IMU (mpu6050)
// 2. Compute the inertia contribution: I * w (w consists of angular rotations)
// 3. Compute the correcting contributions: using pid combined with pose information
// 4. Update wrench portion
task computeDesiredOrientation([&drone, &wrench]() {
  drone.sensor<mpu6050>().read();

  matrix inertiaContribution = drone.profile->getInertiaMatrix() * matrix({
    {drone.pose.roll()},
    {drone.pose.pitch()},
    {drone.pose.yaw()}
  });

  matrix pidContribution({
    {drone.profile->getMass() / 40 * 3.55 * -drone.pose.pitch()},
    {drone.profile->getMass() / 40 * 3.55 * -drone.pose.roll()},
    {0.55 * -drone.pose.yaw()}
  });

  wrench.set(4, 1, 6, 1, inertiaContribution + pidContribution);
}, 5, 1);

// Compute desired acceleration (rotational forces tau_x, tau_y, tau_z) every 5 ms 
// 1. Read the IMU (mpu6050)
// 2. Compute world-relevant acceleration to 3x1 vector. (raw acceelration changes with angle)
// 3. Update wrench portion
// [NOTE] here, the correct acceleration values are not used—yet.
task computeDesiredAcceleration([&drone, &wrench]() {
  drone.sensor<mpu6050>().read();
  
  matrix accellerationContribution({
    {drone.pose.ax()},
    {drone.pose.ay()},
    {drone.pose.az()}
  });

  wrench.set(1, 1, 3, 1, accellerationContribution);
}, 5, 1);

// Compute necessary thrusts based on desired orientation and acceleration (wrench)
// 1. Compute the motor thrusts: A⁻¹ * F
//  | We use the pre-computed inverse of A times the wrench to retrieve M.
//  | For rectangular matrices, the pseudoinverse is calculated instead of a regular inverse.
task computeMotorThrusts([&drone, &wrench, &motorThrusts]() {
  motorThrusts = drone.profile->getInverseAllocationMatrix() * wrench;
}, 5, 0);

// [Temporary task] Compute and display barometer readings
// [NOTE] Task will be used in the future for approximate altitude estimation.
task printBarometerData([&drone]() {
  Serial.print("Temperature: ");
  Serial.println(drone.sensor<bmp280>().read().temperature());
  Serial.print("Air density: ");
  Serial.println(drone.sensor<bmp280>().read().density());
  Serial.print("Pressure: ");
  Serial.println(drone.sensor<bmp280>().read().pressure());
  Serial.print("Approx. Altitude: ");
  Serial.println(drone.sensor<bmp280>().read().altitude());
}, 1000, 1);

// [Temporary task] Tests for new `drone_event_t` status messages
task testErrorMessages([&drone]() {
  drone_event_t info(drone_event_t::notice, "Home updated.");
  drone_event_t warning(drone_event_t::caution, "GPS signal lost.");
  drone_event_t error(drone_event_t::error, "Aircraft pitch angle too large.");
  drone_event_t criticalError(drone_event_t::critical, "Motor 1 unresponsive.");
  
  Serial.println(info.compose());
  Serial.println(warning.compose());
  Serial.println(error.compose());
  Serial.println(criticalError.compose());
}, 0, 3);

// Initialization sequence (The obvious is ignored)
// 1. Add and construct desired sensors for future use
// 2. Provide and configure thruster interfaces
//  | In this case we use 4 standard ESC's with characteristics:
//  | - <pin>
//  | - minimum PWM speed 1000
//  | - maximum PWM speed 2000
//  | - motor kv rating 2300
// 3. Bind orientation and acceleration change channels to the drone's built-in pose for accessibility*
//  | *Accessibility within the drone class when necessary
// 4. Start up desired sensors
// 5. Calibrate desired sensors
void setup() {
  Serial.begin(115200);

  drone.addSensor<mpu6050>();
  drone.addSensor<bmp280>();

  drone.setThrustControllers<ESC>({
    ESC(13, 1000, 2000, 2300),
    ESC(22, 1000, 2000, 2300),
    ESC(15, 1000, 2000, 2300),
    ESC(26, 1000, 2000, 2300)
  });
  
  drone.pose.bindAx(&drone.sensor<mpu6050>().ax_mps2)
            .bindAy(&drone.sensor<mpu6050>().ay_mps2)
            .bindAz(&drone.sensor<mpu6050>().az_mps2)
            .bindGx(&drone.sensor<mpu6050>().gx)
            .bindGy(&drone.sensor<mpu6050>().gy)
            .bindGz(&drone.sensor<mpu6050>().gz)
            .bindRoll(&drone.sensor<mpu6050>().roll)
            .bindPitch(&drone.sensor<mpu6050>().pitch)
            .bindYaw(&drone.sensor<mpu6050>().yaw);

  drone.sensor<mpu6050>().begin(19, 23);
  drone.sensor<bmp280>().begin(19, 23);

  drone.sensor<mpu6050>().calibrate();

  testErrorMessages();
}

// Main loop (The obvious is ignored)
// 1. Execute task for computing desired orientation
// 2. Execute task for computing desired acceleration
// 3. Execute task for computing required thrusts
// 4. Display necessary thrusts
void loop() {
  computeDesiredOrientation();
  computeDesiredAcceleration();
  computeMotorThrusts();
  
  Serial.print(motorThrusts(1, 1)); // back left
  Serial.print(" ");
  Serial.print(motorThrusts(2, 1)); // front left
  Serial.print(" ");
  Serial.print(motorThrusts(3, 1)); // front right
  Serial.print(" ");
  Serial.println(motorThrusts(4, 1)); // back right
}
