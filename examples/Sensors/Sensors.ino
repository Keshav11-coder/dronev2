/** © 2025 Keshav Haripersad
 *  dronev2 - Sensors
 * 
 *  Licensed under the MIT license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/dronev2
 */

// Library setup
// 1. For this example, we will use 2 built-in libraries: mpu6050 (imu) and bmp280 (barometer)
// 2. The sensor integration is built-in
// 3. Use the namespace dronev2 to prevent messy code
#include <dronev2.h>
#include <dronev2/mpu6050.h>
#include <dronev2/bmp280.h>
using namespace dronev2;

// This system looks like a lot at first, but is actually very simple.
// In other examples, the Drone class is initialized as such:
Drone<> example;

// This is the basic setup, But if you want to use other classes from within your drone (to keep systems centralized), we should make use of that '<>'
// The <> represents a template. Templates allow us to define types at a function call or object initialization. Using templates allows us 
// to accept a sensor of any type at addition, as long as it is included here.
// So in conclusion, all this does is 'import' sensors (or devices e.g. camera's) the drone should know about. These can be used during addition.
// Here's an example of such a constructor, utilizing the mpu6050 and bmp280 sensors:
Drone<mpu6050, bmp280> aircraft;

// Let's continue by actually adding these sensors to our available sensors list.
// This can get a bit tougher, but if you understand the templates concept this will be self-explanatory.
void setup() {
  aircraft.addSensor<mpu6050>();

  // Now—that looks terrifying, but let's run through it.
  // We first call the built-in addSensor method of 'Drone', and we provide the type of sensor we want to use.
  // In this case, we want to use the mpu, so we just put that type between the template arguments (<>).
  // Now, this is an important thing. Some sensors have different kinds of constructors than others, which is why this featrue exists—to allow users to add any kind of sensor.
  // See the parenthesis after the template arguments. See this as the constructor tuple you would use regularly.
  // So, for something like `mpu6050 mympu(21, 22)`, you would take those arguments (21, 22) and just place them in the place of the original parenthesis:
  // aircraft.addSensor<mpu6050>(21, 22);

  // Getting a sensor
  // To use these sensors through the drone interface, use this syntax:
  aircraft.sensor<mpu6050>(); // returns that sensor as object

  // What we did here is similar to addSensor. We basically requested, of all sensors, the sensor with type mpu6050.
  // Don't forget the (), those are required. Then you can immediately call any method of that sensor.

  // This type-based sensor retrieving system can introduce some clashes between multiple sensors of the same type, which is why we're working on a system where you can get
  // a sensor by type and specify its index. If no index is given, the function picks the very first. Example:
  // There are two camera's (does not compile, is an example):
  // aircraft.sensor<camera>(1) => returns the first camera
  // aircraft.sensor<camera>(2) => returns the second camera
  // aircraft.sensor<camera>() => (default) returns the first camera

  // It might look excessive at first, but it's the small price you pay for having any type of sensor available for use. Using std::any or similar systems could be memory-heavy for such small MCU's.
}

void loop() {}
