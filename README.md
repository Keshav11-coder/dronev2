![License](https://img.shields.io/github/license/Keshav11-coder/dronev2?label=license)
![Issues](https://img.shields.io/github/issues/Keshav11-coder/dronev2)
![Release](https://img.shields.io/github/v/release/Keshav11-coder/dronev2)
![Commit Activity](https://img.shields.io/github/commit-activity/m/Keshav11-coder/dronev2)

# `dronev2` **|** `v1.1.0`
A simple, modular drone library designed to simplify drone programming. While offering flexibility, the library also provides high-level control systems and easy customization—all while keeping the syntax clean and readable. Additionally, we offer a fully extensible design, ideal for developers seeking a structured, efficient way to implement advanced flight control systems.

## Table of Contents

1. [Introduction](#dronev2--v110)  
2. [Get Started](#get-started)  
3. [API Guide](#api-guide)  
    - [Basic Setup](#basic-setup)  
    - [Profile Initialization](#profile-initialization)  
      - [Parameters and Descriptions](#available-parameters-and-descriptions)  
      - [Profile Setup](#profile-setup)  
    - [Sensors, Devices, and Peripherals](#sensors-devices-and-peripherals)  
      - [Enabling Peripherals](#enabling-peripherals)  
      - [Adding Peripherals](#adding-peripherals)  
      - [Retrieving Peripherals](#retrieving-peripherals)  
    - [Thruster Initialization](#thruster-initialization)  
    - [Tasks and Threads](#tasks-and-threads)  
      - [Tasks](#tasks)  
      - [Threads](#threads)  
    - [Matrix Operations](#matrix-operations)  
4. [Supported Hardware](#supported-hardware) 

## Get started
To start developing with `dronev2`, follow these steps:
* Download the repository as a zip from github.
* Add the library to the Arduino IDE using **Sketch > Include library > Add .zip library**.
* Finally, you can take a look at some of the examples or follow the API guide for more information.

> [!NOTE]
> Make sure you have the ESP32 boards installed in the Arduino IDE. Make sure you have a minimum board version of **3.1.0**, which supports `C++17`.

---

## API guide
Navigating through this library might seem tricky at first, but if you follow this detailed API guide, you'll easily be able to use the library effectively.

### Basic setup
`dronev2` is highly modular and extensible, so it does not rely on any large modules to operate on its own. At its most basic, the library can be initialized like so:

```cpp
#include <dronev2.h>

Drone<> myDrone;

void setup() {}
void loop () {}
```

The `Drone` class is primarily used to keep functionality and parameters centralized.

> [!TIP]
> All built-in `dronev2` objects and methods are inside of the `dronev2` namespace, so to avoid messy code consider including the namespace as whole as well:
> ```cpp
> using namespace dronev2;
> ```
> Only consider doing this if you're sure no methods will clash with third-party methods.

---

### Profile initialization
Profiles can be created to store drone frame configurations easily in an object. You can create one using `Profile` and pass it to a drone constructor.
The `Profile` class has two constructors. Both are demonstrated below.

#### Available parameters and descriptions
| **Parameter**             | **Type**                 | **Description**                                                                                  |
|---------------------------|--------------------------|--------------------------------------------------------------------------------------------------|
| `A` (Allocation Matrix)   | `matrix`                | A matrix that defines the thrust distribution for the drone's motors.                           |
| `I` (Inertia Matrix)      | `matrix`                | A matrix that represents the moment of inertia for the drone's body.                            |
| `m` (Mass)                | `float`                 | The total mass of the drone.                                                                    |
| `a` (Arm Length)          | `float`                 | The length of each arm connecting the motor to the center of the drone.                         |
| `pd` (Propeller Diameter) | `float`                 | The diameter of the propeller used on the drone.                                                |
| `pp` (Propeller Pitch)    | `float`                 | The distance a propeller would travel in one rotation, assuming no slippage.                   |
| `kv` (Motor Kv Rating)    | `int`                   | The motor's Kv rating, which specifies the RPM per volt applied.                                |
| `V` (Operating Voltage)   | `float`                 | The operating voltage of the drone's motor system.                                              |

#### Profile setup
```cpp
// Descriptive
Profile configuration = Profile()
  .setAllocationMatrix(matrix({{...}}))
  .setInertiaMatrix(matrix({{}}))
  .setMass(float)
  .setArmLength(float)
  .setPropellerDiameter(float)
  .setPropellerPitch(float)
  .setMotorKvRating(int)
  .setOperatingVoltage(float);

// Quick and compact
Profile configuration(A, I, m, a, pd, pp, kv, V);

// Create `Drone` using the configuration
Drone<> myDrone(&configuration); // Requires a pointer of Profile
```
> [!TIP]
> Both the `Profile` and `matrix` classes are supported by default.

---

### Sensors, devices and peripherals
As mentioned before, `dronev2` is highly extensible, which is what the templated class is used for. This system allows users to easily import any third-party classes and systems they would like to use, without the need for base classes. The templated system is a small price you pay to have an extensible design, while maintaining type-safety and efficiency. Further optimizations will be made in the future, but this is as far as we got.

#### Enabling peripherals
```cpp
// Third-party sensors
#include <mpu6050.h>
#include <bmp280.h>

Drone<mpu6050, bmp280> myDrone(&configuration);
```

#### Adding peripherals
Classes (sensors) of which the type has been specified in the template argument list may then be used in the `addSensor` method of `Drone`:

```cpp
myDrone.addSensor<mpu6050>(Args...);
```

> [!NOTE]
> `Args...` are the arguments of the constructor of the type specified in the template.

#### Retrieving peripherals
Getting the sensor works similarly. This method returns the object of the sensor by requested type (in the template)—if available.

```cpp
myDrone.sensor<mpu6050>();
```

> [!IMPORTANT]
> In cases of multiple devices, such as cameras, type retrieving can cause clashes. In a future update, you will be able to access a device by type and by index:
> 
> ```cpp
> myDrone.sensor<camera>(1); // Gets the first of two cameras
> 
> myDrone.sensor<camera>(2); // Gets the second of two cameras
>
> myDrone.sensor<camera>(); // If no index is specified, it chooses the first.
> ```

---

### Thruster initialization
Unlike sensors, thrusters require a certain set of methods which make them a bit less flexible. In order to keep consistent with our failsafe systems and future development plans, we have decided to go with a **base-to-inherited class system**. However, we have still decided to keep the template argument in case we find a solution to allow for more flexibility. The base class for thrusters is `ThrusterInterface`, and one of the built-in modules `dronev2/ESC` inherits from that class.

```cpp
#include <dronev2/ESC.h> // Built-in ESC class, inherits from `ThrusterInterface`

Drone<> myDrone(&configuration); 

myDrone.setThrustControllers<ESC>({
  ESC(13, 1000, 2000, 2300) // pin, min, max, kv
  ... 
});
```

The method accepts an initializer list (array) of the type specified in the template. This provides developers with room to consider future improvements that may reduce or eliminate these requirements.

---

### Tasks and threads
The `dronev2` library provides built-in modules for tasks. These classes are just regular functions that have an interval (if looped), and are non-blocking, making them ideal for quick setups. The tasks accept lambdas as well as regular void functions. 

Threads simply combine tasks to avoid clusters of function calls in the loop.

#### Tasks

```cpp
#include <dronev2/tasker.h>

// Lambda task, runs every second, has an id of 1.
task task1([]() { // Capture any variables you will need in the [], e.g. [&myDrone]
  // Your logic here
  // ...
}, 1000, 1);

// Void-accepted task, runs every 2 seconds, has an undeclared id.
void function() {}

task task2(function, 2000);

void loop() {
  task1(); // You can also use task1.execute(); if you prefer to do so.
  task2();
}
```

> [!TIP]
> Tasks can be called once as well. This requires the interval to be set to 0 to ensure the runner executes the task.

#### Threads
As mentioned before, threads combine tasks and prevent clutter in the loop.

```cpp
thread thread1(task1, task2);

void loop() {
  thread1.run();
}
```

Alternatively, if you want to provide pointers to tasks instead, use thread_ptr. It has the same functionality, but just accepts pointers to tasks.

---

### Matrix operations
`dronev2` supports the `matrix` class by default for high-level operations, which can be found [here](https://github.com/Keshav11-coder/matrix.h).

---

## Supported hardware
This library is primarily written to operate on embedded systems using ESP32. The library files were compiled and tested on the ESP32 DEVKIT V1 board, ***using the ESP32 board version 3.1.0***. The reason we use the latest version is to generally keep up with updates and for the advanced C++17 features that are not available in the older versions.

## Conclusion
`dronev2` is a highly modular, high-level drone library that simplifies drone programming. Our library balances high-performance operations with a structured environment, clean syntax, and readability. Our system is easily extensible to allow for custom (third-party) objects for close to full flexibility.

Improvements and optimizations are consistently made, so look out for updates.

We apologize in advance for any long intervals between releases.
