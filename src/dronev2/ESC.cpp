/** © 2024 Keshav Haripersad
 *  Basic ESC class - ESC.cpp
 *  This class inherits from ThrusterInterface (check dronev2.h)
 *  
 *  This class was written for typical ESCs, and is not intended for other types of thrusters.
 */

#include "ESC.h"

namespace dronev2
{
    bool ESC::failsafe_system::engage()
    {
        failsafe = true;
        return true;
    }

    bool ESC::failsafe_system::disengage()
    {
        failsafe = false;
        return true;
    }

    bool ESC::failsafe_system::status()
    {
        return failsafe;
    }

    int ESC::arm()
    {
        if (!failsafe.status())
        {
            esc.writeMicroseconds(esc_min);
            armed = true;
            return 0;
        }
        return 1;
    }

    int ESC::stop()
    {
        if (armed && !failsafe.status())
        {
            esc.writeMicroseconds(esc_min);
            failsafe.engage();
            return 0;
        }
        return 1;
    }

    int ESC::restart()
    {
        if (armed && failsafe.status())
        {
            esc.writeMicroseconds(esc_min);
            failsafe.disengage();
            return 0;
        }
        return 1;
    }

    int ESC::max()
    {
        return esc_max;
    }

    int ESC::min()
    {
        return esc_min;
    }

    int ESC::write(float spd) 
    {
        if (armed && !failsafe.status())
        {
            if (spd >= esc_min && spd <= esc_max)
            {
                esc.writeMicroseconds(static_cast<int>(spd));
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