/** © 2024 Keshav Haripersad
 *  Base API logic - dronev2.cpp
 *  | Check out dronev2.h for an overview.
 *  | Check out files in /dronev2 for other modules.
 * 
 *  Licensed under the MIT license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/dronev2
 */

// Headers
#include "dronev2.h"

// For matrix operations
#include "dronev2/tools/matrix.h"

// Logic (No walkthrough required, getter/setter logic)
namespace dronev2
{
    matrix Profile::getAllocationMatrix()
    {
        return _AllocationMatrix;
    }

    matrix Profile::getInverseAllocationMatrix()
    {
        return _InverseAllocationMatrix;
    }

    matrix Profile::getInertiaMatrix()
    {
        return _InertiaMatrix;
    }

    float Profile::getMass()
    {
        return _Mass;
    }

    float Profile::getPayloadCapacity()
    {
        return _Payload;
    }

    float Profile::getPropellerDiameter()
    {
        return _PropellerDiameter;
    }

    float Profile::getPropellerPitch()
    {
        return _PropellerPitch;
    }

    float Profile::getArmLength()
    {
        return _ArmLength;
    }

    float Profile::getMotorKvRating()
    {
        return _MotorKvRating;
    }

    float Profile::getOperatingVoltage()
    {
        return _OperatingVoltage;
    }

    // SET functions
    Profile &Profile::setAllocationMatrix(matrix allocation_matrix)
    {
        _AllocationMatrix = allocation_matrix;
        if (_AllocationMatrix.rows() == _AllocationMatrix.columns()) // Square matrix, use regular inverse
        {
            _InverseAllocationMatrix = !_AllocationMatrix;
        }
        else // Non-square matrix (hexacopters, octocopters)
        {
            _InverseAllocationMatrix = _AllocationMatrix.pseudoinverse(); // Pseudo-inverse returns a matching matrix
        }
        return *this;
    }

    Profile &Profile::setInverseAllocationMatrix(matrix inverse_allocation_matrix)
    {
        _InverseAllocationMatrix = inverse_allocation_matrix;
        return *this;
    }

    Profile &Profile::setInertiaMatrix(matrix inertia_matrix)
    {
        if (inertia_matrix.rows() == 3 && inertia_matrix.columns() == 3)
        {
            _InertiaMatrix = inertia_matrix;
        }
        else
        {
            _InertiaMatrix = matrix(3, 3, 1);
        }
        return *this;
    }

    Profile &Profile::setMass(float mass)
    {
        _Mass = mass;
        return *this;
    }

    Profile &Profile::setPayloadCapacity(float payload_capacity)
    {
        _Payload = payload_capacity;
        return *this;
    }

    Profile &Profile::setPropellerDiameter(float propeller_diameter)
    {
        _PropellerDiameter = propeller_diameter;
        return *this;
    }

    Profile &Profile::setPropellerPitch(float propeller_pitch)
    {
        _PropellerPitch = propeller_pitch;
        return *this;
    }

    Profile &Profile::setArmLength(float arm_length)
    {
        _ArmLength = arm_length;
        return *this;
    }

    Profile &Profile::setMotorKvRating(float kv)
    {
        _MotorKvRating = kv;
        return *this;
    }

    Profile &Profile::setOperatingVoltage(float voltage)
    {
        _OperatingVoltage = voltage;
        return *this;
    }
}