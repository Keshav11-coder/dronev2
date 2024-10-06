// Header motor.h
#include "motor.h"

namespace dronev2
{
    motor::motor(int pin, int min, int max)
        : esc_pin(pin), esc_min(min), esc_max(max)
    {
        esc.attach(pin);
    }

    bool motor::failsafe_system::engage()
    {
        failsafe = true;
        return failsafe;
    }

    bool motor::failsafe_system::disengage()
    {
        failsafe = false;
        return failsafe;
    }

    bool motor::failsafe_system::status()
    {
        return failsafe;
    }

    int motor::arm()
    {
        if (!failsafe.status())
        {
            esc.writeMicroseconds(esc_min);
            armed = true;
            return 0;
        }
        return 1;
    }

    int motor::stop()
    {
        if (armed && !failsafe.status())
        {
            esc.writeMicroseconds(esc_min);
            failsafe.engage();
            return 0;
        }
        return 1;
    }

    int motor::restart()
    {
        if (armed && failsafe.status())
        {
            esc.writeMicroseconds(esc_min);
            failsafe.disengage();
            return 0;
        }
        return 1;
    }

    int motor::max()
    {
        return esc_max;
    }

    int motor::min()
    {
        return esc_min;
    }

    int motor::write(int spd)
    {
        if (armed && !failsafe.status())
        {
            if (spd >= esc_min && spd <= esc_max)
            {
                esc.writeMicroseconds(spd);
                return 0;
            }
            else
            {
                return 1;
            }
        }
        return 1;
    }
}