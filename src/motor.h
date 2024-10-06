#ifndef dronev2_motor_h
#define dronev2_motor_h

#include "ESP32Servo.h"

namespace dronev2
{
    struct motor
    {
    private:
        Servo esc;
        int esc_pin;
        int esc_min;
        int esc_max;
        bool armed = false;

    public:
        motor(int pin, int min, int max);

        struct failsafe_system
        {
        private:
            bool failsafe = false;

        public:
            bool engage();
            bool disengage();
            bool status();
        } failsafe;

        int arm();
        int stop();
        int restart();
        int max();
        int min();

        int write(int spd);
    };
}

#endif
