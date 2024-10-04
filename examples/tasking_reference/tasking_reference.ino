#include <dronev2.h>
#include <Wire.h>

/**
 * This example shows the basic use of the tasking mechanism.
 *  - Explaining tasking mechanisms on various complexity levels.
 *  - Explaining how the tasking actuall works in looping.
 *
 * Note: When reading this example's documentation, the writer presumes you have read the basic_reference.ino first to get a grasp of the system.
 */

// Define your drone instance
dronev2::drone quadcopter;

// For the examples
int threshold = 0.95;

void setup()
{
    Serial.begin(115200);

    randomSeed(analogRead(0));

    // Pinmode setup for the LED tasks.
    pinMode(2, OUTPUT);
    pinMode(4, OUTPUT);

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

        // Imu initialization is fairly simple as well. Though we don't yet support custom TwoWire objects, you can specify your sda and scl pins respectively for now.
        // This way you still have some kind of customization, and if you wish to go more advanced you can use tasks (see other examples) to interface with the imu in a different way.
        .imu(19, 23);


    /** Tasking & Threading API
      * - Creating tasks
      * - Extending functionality with lambda's
      * - Executing tasks
      * - Combining tasks using threads
      * - Creating and using threads
    */

    // Creating a task is very simple. Since the system has built-in intervals that are adjustable, we can go many ways from here. 

    // Here's a task with 50 milisecond interval rate.
    dronev2::task task1([](){

    }, 50);

    // Since tasks use lambda functions (google if you're a berginner), we can capture anything in it's variables and use it. We can also restrict variable pickups.
    // Multiply a static variable by a captured threshold every second.
    dronev2::task increment([threshold](){
        static int variable = 1;
        variable = variable*threshold;
    }, 1000);

    // Now that we know how to initialize tasks, we can try executing them.
    // To execute a task, we simply call task_instance.execute(). Check it out:
    task1.execute();
    increment.execute();

    // But this won't execute unless we specify it properly in a loop, and to make it non-blocking let's use threading.
    // Now, threading is not a standard c++ thing, so we kind of "mimick" this system. Check out the source code in dronev2.cpp if you feel like it.

    // We can add all tasks at setup, or later too.
    dronev2::thread threads(task1); // or more! (task1, task2, ..., taskN)

    // Add the increment task later on.
    threads.add_task(increment);

    // To execute a thread, just call threads.run(). I won't do it here because, again, there's no use unless done in a loop.

    // To clear a thread, use the respectvie function name:
    threads.clear();

    
    /** The drone's built-in thread
      * - The built-in thread
      * - How to utilize the built-in thread.
      * - How to properly execute the thread.
    */

    // The drone has a default thread to which you can add to or remove tasks from. You can interface it like this:
    // Adding a task to the thread:

    quadcopter.add_task(new dronev2::task([](){
        static bool ledState2 = LOW;
        ledState2 = !ledState2;
        digitalWrite(4, ledState2); 
    }, 50));

    // This is a very simple blink system that keeps track of the toggle state. Its the same logic as before but then we use the new keyword to make it more compact.


    // Notes:
    //  - When tasks get cleared, their memory also does. There might be some faults here and there, but  it has worked perfectly fine (long duration) so far.
    //      If any issues come up, please open an issue on github.
}

// The loop introduces one line.
void loop()
{
    // In the past we had nothing in here, but for tasks to work as a "scheduling" system, we need to loop so the tasks can schedule in a non-blocking manner, using the infamous millis() method.
    // All we have to do is run quadcopter.persist_tasks();
    // Since the drone has a built-in thread already, all this function does is run that thread using the persist_tasks function name as a wrapper function to protect the private thread.
    quadcopter.persist_tasks();
}