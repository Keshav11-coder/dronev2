/** © 2024 Keshav Haripersad
 *  Basic BMP280 class - bmp280.cpp
 *  
 *  This class was written for typical BMP's, and is a wrapper for the Adafruit BMP280 library.
 */

// Uses adafruit
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#include "bmp280.h"

namespace dronev2
{
    int bmp280::begin(int sda, int scl)
    {
        // Initialize I2C with custom SDA and SCL pins
        Wire.begin(sda, scl);

        // Initialize BMP280 sensor
        if (!bmp.begin(0x76))
        { // 0x76 is a common I2C address for BMP280
            return 1;
        }

        // Configure BMP280
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X2,     // Temperature oversampling
                        Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                        Adafruit_BMP280::FILTER_X16,      // Filtering
                        Adafruit_BMP280::STANDBY_MS_500); // Standby time
        return 0;
    }

    void bmp280::calibrate(float knownPressure)
    {
        // Simple calibration: adjust the pressure reading based on a known reference
        // You could implement a more complex calibration if necessary
        float calibrationOffset = knownPressure - pressure();
        _pressure += calibrationOffset; // Adjust internal pressure reading
    }

    float bmp280::temperature()
    {
        return _temperature;
    }

    float bmp280::altitude()
    {
        return bmp.readAltitude(1013.25); // in hPa
    }

    float bmp280::pressure()
    {
        return _pressure;
    }

    float bmp280::density(){
        return (_pressure*100.0F) / (287.05*(_temperature+273.15));
    }

    bmp280 &bmp280::read()
    {
        _temperature = bmp.readTemperature();
        _pressure = bmp.readPressure() / 100.0F;
        return *this; // Chainable, e.g.

        /*

        mybmp.read().temperature()
        mybmp.read().pressure()

        */
    }

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