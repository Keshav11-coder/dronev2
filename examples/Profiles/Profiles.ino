/** © 2025 Keshav Haripersad
 *  dronev2 - Profiles
 *  
 *  Note: This feature is built for advanced users who are familiar with more advanced mathematical and physics concepts.
 * 
 *  Licensed under the MIT license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/dronev2
 */

// Library setup
// 1. Profiling is supported by default
// 2. Use the namespace dronev2 to prevent messy code
#include <dronev2.h>
using namespace dronev2;

// Creating a profile object
// 1. You can use the prototype constructor to allow for setup later on
// 2. You can use the other constructor for advanced, initial setup
Profile profile;
// Profile advanced(allocation matrix, inertia matrix, mass, payload capacity, propeller diameter, propeller pitch, arm length, motor kv rating, operating voltage);

// Profile setup
// 1. Set the allocation matrix
// 2. Set the inertia matrix
// 3. Set the mass in kilograms
// 4. Set the payload capacity in kilograms
// 5. Set the propellor diameter in meters
// 6. Set the propeller pitch in meters
// 7. Set the arm length in meters
// 8. Set the motor Kv rating
// 9. Set the operating voltage
// [NOTE] Profile setup is done with set-methods for clear, descriptive code
// [TIP] This way of setting up is very helpful, as you are allowed to provide only necessary values. These values may change later on.
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

// Using a profile with a Drone object
// 1. Create a regular drone instance and pass the configuration (Profile) in as pointer.
// [NOTE] The <> you see after 'Drone' are required, because drone is templated. The sensor example explains why this is in detail.
Drone<> aircraft(&configuration);

// Getting profile values
// 1. Use regular 'get' keywords with their respective names
// 2. Get values via the drone instance. Requires the pointer accessor ->
void setup(){
  matrix A = configuration.getAllocationMatrix();
  float m = configuration.getMass();

  matrix B = aircraft.profile->getAllocationMatrix();
  // etc ...
}

void loop() {}
