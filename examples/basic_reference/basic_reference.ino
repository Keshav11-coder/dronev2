#include <dronev2.h>
#include <Wire.h>

/**
 * This example shows the basics on how to use the drone library.
 *  - Explaining the initialization steps for basic drone initialization
 *  - Explaining the syntax for writing to motors.
 *
 */

// Define your drone instance
dronev2::drone quadcopter;

void setup()
{
    Serial.begin(115200);

    // Important! This line's importance depends on which features you use.
    // When using the tasking system, this line is important for generating task id's.
    // Id you don't specify this you have nothing to worry as the system will still work. 
    // If you choose not to enable it, the id's will be predictable for each boot. (Which is what most people prefer).

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
        
        // Imu initialization is fairly simple as well. Though we don't yet support custom TwoWire objects, you can specify your sda and scl pins respectively for now.
        // This way you still have some kind of customization, and if you wish to go more advanced you can use tasks (see other examples) to interface with the imu in a different way.
        .imu(19, 23);

    // Basic speed writing
    // Writing speeds to your drone is very simple now. You can either use write_all or it's variation for selective writing.

    // First, arm the motors.
    quadcopter.arm_all();
    delay(5000);

    // Then we can write a basic low speed to all motors.
    quadcopter.write_all(1050);
    delay(5000);

    // Eventually we stop after 5 seconds.
    quadcopter.write_all(1000);

    // For selective speed writing, you can use the same .write_all() function, but then using any amount again. Check it out:
    // This mini example shows the flexibility and ease of use for the write function:
    quadcopter.write_all(1000, 1050, 1000, 1050);
    delay(3000);
    quadcopter.write_all(1000);

    // Notes: 
    //  - As you might have noticed, the speed writing is actually a state write. So whatever you set the motor speed to, it will stay like that until you change it or disarm.
    //  - The selective writing expects that the amount of arguments is equal to the amount of motors specified in the initialization. Be careful how you use this. If the arguments do not match they won't write.
}


// The loop is empty keeping the code neat.
void loop()
{
}