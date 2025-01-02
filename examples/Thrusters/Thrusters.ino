/** © 2025 Keshav Haripersad
 *  dronev2 - Thrusters
 * 
 *  Licensed under the MIT license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/dronev2
 */

// Library setup
// 1. Thruster setup is supported by default. However, we will use an external, built-in library to showcase this.
// 2. Use the namespace dronev2 to prevent messy code
#include <dronev2.h>
#include <dronev2/ESC.h> // external
using namespace dronev2;

// Create our drone instance
Drone<> aircraft;

// Just like the sensors, the thrusters also use templates to identify which thruster is used.
// However, the thruster class must currently be inherited from a built-in base class called ThrusterInterface. 
// This is because of several reasons:
// 1. Required functions for built-in control functions, such as write, failsafe, arm etc...
// 2. Allowing any kind of thruster interfaces might introduce runtime memory and thruster risks.

// We might find a work-around for this, allowing for multiple kinds of types in the template for different kidns of interfaces.
// We made the template initiative early so we can possibly extend it when the possibility for optimization for 'any' types becomes available.
// We can still use templates to maintain type safety.

// Adding thrusters
// Unlike sensors, thrusters require a quick, easy and direct setup of all thrusters. That is why we use an array of the specified template type.
// [NOTE] Calling the set thrusters function more than once clears the thrusts set by the last made call.

// We do this to, again, maintain a centralized system.
void setup() {
  aircraft.setThrustControllers<ESC>({
    ESC(13, 1000, 2000, 2300),
    ESC(22, 1000, 2000, 2300),
    ESC(15, 1000, 2000, 2300),
    ESC(26, 1000, 2000, 2300)
  });
}

void loop() {}
