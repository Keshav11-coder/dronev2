![License](https://img.shields.io/github/license/Keshav11-coder/dronev2?label=license)
![GitHub issues](https://img.shields.io/github/issues/Keshav11-coder/dronev2)
![GitHub release](https://img.shields.io/github/v/release/Keshav11-coder/dronev2)
![GitHub commit activity](https://img.shields.io/github/commit-activity/m/Keshav11-coder/dronev2)

# Table of Contents
1. [License](https://github.com/Keshav11-coder/dronev2?tab=MIT-1-ov-file)
2. [Release Information](#release-information)
    - [Basic Features](#basic)
    - [More Advanced Features](#more-advanced)
    - [The Drone Class](#the-drone-class)
3. [Setup & Usage](#dronev2-setup--usage)
    - [Installation in the Arduino IDE](#installation-in-the-arduino-ide)
    - [Get Started](#get-started)
    - [Declare Your Drone Instance](#declare-your-drone-instance)
    - [Initialize Your Drone](#initialize-your-drone)
4. [Using the Built-in PID System](#using-the-built-in-pid-system)
    - [Make Your PID Instance](#make-your-pid-instance)
    - [Execute the PID System](#execute-the-pid-system)
    - [Adjust PID Values Runtime](#adjust-pid-values-runtime)
5. [Tasking & Threading](#tasking--threading)
    - [The Default Thread](#the-default-thread)
    - [Basic Usage](#basic-usage)
    - [Combining Tasks](#combining-tasks)

# `dronev2` *Modular release* - v1.0.1
`dronev2`, a drone interfacing library which aims to improve the way we program drones. The library features a modular and structural design, making it easier for new developers to start programming their drones. 

The library does not only focus on drone programming, but also offers a fairly simple tasking/scheduling system which could be useful in new projects.

We finally decided to split the main codebase which originally consisted of `dronev2.h` and `dronev2.cpp` into multiple parts with their respective classes or structs. All modules still stay under the namespace `dronev2` to have most previous codebases still working after this change.

The library is licensed under the MIT license. This was chosen over the other open-source licenses because we offer functionality. If you're here for something specific, you can take that.

Be on the lookout for future versions that introduce more modules! 

## Release information
This release splits the library into multiple different modules for easy interfacing and to have a more straightforward modular design.

### Basic 
- **Motors** (*`struct`* `motor`): A pretty basic motor class that allows you to do basic arming and writing. It also includes a built-in `failsafe` system.
- **PID systems** (*`struct`* `pid`): A fairly simple PID system that allows for basic functions such as `run`, `adjust` (modifying values) and `reset`.

### More advanced
- **Tasks** (*`class`* `task`): This is part of a modular system. It is a singular task that belongs to a thread, or can be executed individually if you prefer it like that. It includes a built-in `interval` system for periodic execution. The only downside is that this system needs its execution to happen inside of an existing loop, requiring a bit more handling in the functions. The given function can also be `lambda`, because it uses `std::function`.
- **Threads** (*`class`* `thread`): Requires `task` to operate. This is basically a collection of scheduled tasks. It offers some basic threading functions, such as `add_task`, `remove_task`, `update_task_interval` (for more advanced systems), `clear` and also requires the function `run` to be executed continuously in a loop. Aside from that it allows you to initialize the thread instance with default tasks inside the constructor. This is managed by a variadic template so it can be any amount.
- **Threads for pointers** (*`class`* `thread_ptr`): Almost the same as `thread`, but instead of task objects it requires task pointer objects. This is used in the `drone` class' main thread.
### The drone class
- *`class`* `drone`: The base class for the drone. It combines most of the previous functionalities into one. The setup is pretty simple: you declare the object, and then specify your `motors`, (optional) `stabilization` & `imu` in the chain. See the API usage in code blocks below for a more detailed reference.

# `dronev2` Setup & usage
Here I explain in detailed code blocks the basic functionalities of `dronev2`.
> [!NOTE]
> This is a very simple documentation. Consider checking out the examples for a more detailed description.


## Installation in the Arduino IDE
The installation of this library is simple. Like any other zip library, download the zip of this repository and add it using the `add .ZIP library` functionality in sketch > include library > add .ZIP library.

## Get started
To get started, include the library like this:
```cpp
#include <dronev2.h>
```


> [!TIP]
> If you don't constantly want to use `dronev2::` in front of each method, try including the namespace as whole:
> ```cpp
> using namespace dronev2;
> ```


### Declare your drone instance
```cpp
dronev2::drone quadcopter;
```

### Initialize your drone
```cpp
void setup() {
   quadcopter
      
      // Set up your motors (pin, min, max)
      .motors(
         new dronev2::motor(4, 1000, 2000),
         new dronev2::motor(5, 1000, 2000),
         new dronev2::motor(15, 1000, 2000),
         new dronev2::motor(16, 1000, 2000)
      )
      
      // Set up your stabilization
      .stabilization([](dronev2::imu_data_packaged imu_data) {
         // Do something with imu_data
      }, 5) // Small stabilization rate.
      
      // Set up your imu (sda, scl)
      .imu(19, 23);
}

void loop() { } // Empty for now.
```


## Using the `pid` system
### Make your PID instance
```cpp
dronev2::pid pid1({3.55, 0.005, 0.0001}); // accepts an std::vector<float>.
```

### Execute the PID system
```cpp
void loop() {
   float actual = 78; // Use actual value here.
   float target = 0; // Use actual target here.
   pid1.run(actual, target);
}
```

### Adjust PID values runtime
```cpp
void loop() {
   pid1.adjust({1, 2, 3}); // Accepts an std::vector<float>. Use actual values here.
```

## Tasking & Threading
For this system you can either use the built-in drone thread, or a custom thread.

### The default thread
The default thread is defined in an instance of `dronev2::drone`, with wrapper functions in the `drone` class to protect the `thread` object inside.

> [!IMPORTANT]
> I suggest checking out one of the guided examples to get a better grasp of the tasking system.

### Basic usage
```cpp
// Creating an instance
dronev2::task task1([]() {
   // Code in here gets executed every 1 second.
}, 1000); // 1 second interval

void loop() {
   // Make sure to add the execute code inside a loop.
   task1.execute(); // Will be executed every 1 second.
}
```

### Combining tasks
```cpp
// Creating tasks
dronev2::task task1([]() {
   // Code in here gets executed every 1 second.
}, 1000); // 1 second interval

dronev2::task task2([]() {
   // Code in here gets executed every 200 milliseconds.
}, 200); // 0.2 second interval

// Add multiple tasks together with a thread
dronev2::thread thread1(task1);

// You can also add them later.
thread1.add_task(task2);

void loop() {
   // Make sure to add the execute code inside a loop.
   thread1.run();
}
```
> [!NOTE]
> The `thread_ptr` class does the same as thread, but then accepts all tasks as pointers only. This might be useful for more compact initialization, or when doing task switching.
