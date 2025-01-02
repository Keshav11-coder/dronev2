#ifndef dronev2_esc_h
#define dronev2_esc_h

/** © 2024 Keshav Haripersad
 *  Basic ESC class - ESC.h
 *  This class inherits from ThrusterInterface (check dronev2.h)
 *  
 *  This class was written for typical ESCs, and is not intended for other types of thrusters.
 */

#include "dronev2.h"
#include <ESP32Servo.h>

namespace dronev2
{
    class ESC : public ThrusterInterface
    {
    private:
        Servo esc;
        int esc_pin;
        int esc_min;
        int esc_max;
        int motor_kv;
        bool armed = false;

    public:
        ESC() {}
        ESC(int pin, int min, int max, int kv) : esc_pin(pin), esc_min(min), esc_max(max), motor_kv(kv) {}

        struct failsafe_system : public ThrusterInterface::failsafe_system
        {
        private:
            bool failsafe = false;

        public:
            bool engage() override;
            bool disengage() override;
            bool status() override;
        } failsafe;

        int arm() override;
        int stop() override;
        int restart() override;
        int max() override;
        int min() override;
        int write(float speed) override; // Ensure this is float
    };
}

#endif
