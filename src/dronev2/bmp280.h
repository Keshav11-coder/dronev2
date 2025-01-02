#ifndef dronev2_bmp280_h
#define dronev2_bmp280_h

/** © 2024 Keshav Haripersad
 *  Basic BMP280 class - bmp280.h
 *  
 *  This class was written for typical BMP's, and is a wrapper for the Adafruit BMP280 library.
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

namespace dronev2
{

    class bmp280
    {
    private:
        Adafruit_BMP280 bmp;

        float _temperature = 0;
        float _pressure = 0;

    public:
        bmp280() : bmp() {}

        int begin(int sda, int scl);

        void calibrate(float knownPressure); // Calibration method

        float temperature();
        float pressure();
        float altitude();
        float density();

        bmp280 &read();
    };

    // Example usage
    /*

    bmp280 sensor;

    void setup()
    {
        Serial.begin(115200);
        sensor.begin(21, 22);
    }

    void loop()
    {
        // Update
        sensor.read();

        // Access seperately
        sensor.temperature();
        sensor.pressure();

        // ... Or access directly

        float temp = sensor.read().temperature();
        float pressure = sensor.read().pressure(); // in hPa
    }

    */
}

#endif