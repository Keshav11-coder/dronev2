/** © 2025 Keshav Haripersad
    dronev2 - Profiles

    Licensed under the MIT license (check LICENSE).
    Github link: https://github.com/Keshav11-coder/dronev2
*/

// Library setup
// 1. The tasking system must be included manually.
// 2. Use the namespace dronev2 to prevent messy code.
#include <dronev2.h>
#include <dronev2/tasker.h>
using namespace dronev2;

// Creating a task
// 1. Tasks accept void functions as well as voided lambda's.
// 2. Tasks can be run once, but are intended for calling inside a loop.
// 3. Tasks support automatic scheduling using miliseconds to allow for non-blocking code.
// 4. Tasks can be given id's (optional, remove that argument if desired).

// [NOTE] Below is an example of a lambda printing task that runs without delay.
task task0([&Serial]() {
  Serial.println("Drone initialized!");
}, 0);

// [NOTE] Below is an example of a lambda printing task that runs every second, and has an id of 0.
task task1([&Serial]() {
  Serial.println("Hello, drone!");
}, 1000, 0);

// [NOTE] Below is an example of a void-based task that runs every 2 seconds, and has no id.
void task2logic() {
  Serial.println("Goodbye, drone!");
}

task task2(task2logic, 2000);

// Combining tasks
// 1. Initialize a thread, and place all tasks inside the argument list.
// 2. For providing pointers to tasks, use thread_ptr instead. Functionality and syntax stays the same.
thread logging(task1, task2);

// Executing tasks once
// 1. Initialize Serial.
// 2. Run the initializing task (once).
// [NOTE] To have a task run once, its interval must be close to zero to avoid delays or else it won't execute at all.
void setup() {
  Serial.begin(115200);

  task0();

  // (You can also use the syntax task0.execute() if you prefer)
}

// Running tasks and threads
// 1. Simply execute or run the desired thread or task
// 2. For threads, use mythread.run().
// 3. For tasks, use the mytask() syntax or use mytask.execute().
// [NOTE] Serial itself might introduce some delay.
void loop() {
  // Uncomment for thread running, comment for task running
  //  logging.run();

  // Uncomment for task running, comment for thread running
  task1();
  task2();
}
