#include <dronev2.h>
#include <Wire.h>

/**
 * This example showcases this library's extensibility and modular design, allowing you to easily integrate third party tools using tasking and threading.
 *  - Demo showing how to use a neural networking system. 
 *  
 *  Note: The neural.h library is not standard with the dronev2.h library. To get that library, check for an embedded version on my github: https://github.com/Keshav11-coder/neural-network
 */

// Our plugin of the day: neural.h. (Easily integratable, easy to use) check it out: https://github.com/Keshav11-coder/neural-network
#include "neural.h"

// Define our plugin (done before the drone to show no immediate relation)
neural::network_v2 nn2(neural::createWeights_mat(9, 1),
                       new neural::layer(
                         neural::activation::Sigmoid,
                         neural::createWeights_mat(4, 9),
                         neural::createWeights_mat(4, 1)),
                       new neural::layer(
                         neural::activation::Softmax,
                         neural::createWeights_mat(4, 4),
                         neural::createWeights_mat(4, 1)));

// Define your drone instance
dronev2::drone quadcopter;

void setup()
{
    Serial.begin(115200);
    randomSeed(analogRead(0));

    // For our led
    pinMode(4, OUTPUT);

    // Drone initialization
    quadcopter

        // Motor initiaization.
        .motors(
            new dronev2::motor(13, 1000, 2000),
            new dronev2::motor(22, 1000, 2000),
            new dronev2::motor(15, 1000, 2000),
            new dronev2::motor(26, 1000, 2000))
        
        // Imu initialization
        .imu(19, 23);

    // An LED for some stress
    quadcopter.add_task(new dronev2::task([](){
        static bool ledState2 = LOW;
        ledState2 = !ledState2;
        digitalWrite(4, ledState2);
    }, 50));

    // Our plugin's (neural.h) integration into the drone's main thread.
    // First capture the neural network's instance
    quadcopter.add_task(new dronev2::task([nn2](){
        // Calculate an output
        std::vector<std::vector<float>> output = neural::feedforward(&nn2);

        // Print it
        printMatrix(output);

        // Specify a target
        std::vector<std::vector<float>> target = {{0.0}, {0.0}, {0.0}, {1.0}};

        // Correct and update the network to achieve that target.
        neural::network_v2 *upd = neural::backpropagate(&nn2, output, target, 0.01);

        // We're not using it so we can probably delete it.
        // delete upd;
    }, 5));
}


void loop()
{
  quadcopter.persist_tasks();
}

// (Optional) PrintMatrix function
void printMatrix(const std::vector<std::vector<float>> &matrix)
{
  Serial.print("{\n");
  for (size_t i = 0; i < matrix.size(); i++)
  {
    Serial.print("  {");
    for (size_t j = 0; j < matrix[i].size(); j++)
    {
      Serial.print(matrix[i][j], 6); // Print with 6 decimal places
      if (j < matrix[i].size() - 1)
      {
        Serial.print(", ");
      }
    }
    Serial.print("}");
    if (i < matrix.size() - 1)
    {
      Serial.println(",");
    }
  }
  Serial.println("\n}");
}

/** Github
 * - check us out: https://github.com/Keshav11-coder/
 * - raise an issue: https://github.com/Keshav11-coder/dronev2/issues
 */