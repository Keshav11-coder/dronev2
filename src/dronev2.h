#ifndef dronev2_h
#define dronev2_h

/** © 2024 Keshav Haripersad
 *  Base API header - dronev2.h
 *  | Check out dronev2.cpp for partial logic.
 *  | Check out files in /dronev2 for other modules.
 *
 *  Licensed under the MIT license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/dronev2
 */

#include <vector>
#include <functional>
#include <memory>
#include <variant>
#include <type_traits>

// For matrix operations (currently unused, testing still in progress)
#include "dronev2/tools/matrix.h"

// Namespace dronev2
namespace dronev2
{
    /*
     * Profile class
     * | Container for model or setup parameters
     * | Used during drone configuration (is accepted in the Drone class constructor)
     * | Overview of parameters:
     * |    - Allocation matrix
     * |    - Inverse allocation matrix
     * |    - Inertia matrix
     * |    - Mass
     * |    - Payload capacity
     * |    - Propeller diameter
     * |    - Propeller pitch
     * |    - Arm length
     * |    - Motor Kv rating
     * |    - Operating voltage
     * |    - (More to come)
     * | Class provides a setter chain for easy and descriptive configuration
     * | Class provides a constructor for advanced users who prefer less unnecessary implementations
     */

    class Profile
    {
    private:
        float _Mass = -1;
        float _Payload = -1;
        float _PropellerDiameter = -1;
        float _PropellerPitch = -1;
        float _ArmLength = -1;
        float _MotorKvRating = -1;
        float _OperatingVoltage = -1;

        // Allocation matrix A
        matrix _AllocationMatrix = matrix(1, 1, 1);
        matrix _InverseAllocationMatrix = matrix(1, 1, 1);
        matrix _InertiaMatrix = matrix(3, 3, 1);

    public:
        // Make it available as prototype
        Profile() {};

        // Recommended constructor
        Profile(matrix A_, matrix I_, float m_, float p_, float p_d_, float p_c_, float a_, float kv_, float V_)
            : _AllocationMatrix(A_),
              _InertiaMatrix(I_),
              _Mass(m_),
              _Payload(p_),
              _PropellerDiameter(p_d_),
              _PropellerPitch(p_c_),
              _ArmLength(a_),
              _MotorKvRating(kv_),
              _OperatingVoltage(V_)
        {
            if (A_.rows() == A_.columns()) // Square matrix, use regular inverse
            {
                _InverseAllocationMatrix = !A_;
            }
            else // Non-square matrix (hexacopters, octocopters)
            {
                _InverseAllocationMatrix = A_.pseudoinverse(); // Pseudo-inverse returns a matching matrix
            }
        }

        // GET functions
        matrix getAllocationMatrix();
        matrix getInverseAllocationMatrix();
        matrix getInertiaMatrix();
        float getMass();
        float getPayloadCapacity();
        float getPropellerDiameter();
        float getPropellerPitch();
        float getArmLength();
        float getMotorKvRating();
        float getOperatingVoltage();

        // SET functions
        Profile &setAllocationMatrix(matrix allocation_matrix);
        Profile &setInverseAllocationMatrix(matrix inverse_allocation_matrix);
        Profile &setInertiaMatrix(matrix inertia_matrix);
        Profile &setMass(float mass);
        Profile &setPayloadCapacity(float payload_capacity);
        Profile &setPropellerDiameter(float propeller_diameter);
        Profile &setPropellerPitch(float propeller_pitch);
        Profile &setArmLength(float arm_length);
        Profile &setMotorKvRating(float kv);
        Profile &setOperatingVoltage(float voltage);
    };

    /*
     * Thruster class
     * | Interface for thruster controllers (e.g. ESCs, Boosters)
     * | Provides a failsafe system for emergency situations
     * | Provides, at its minimum, basic control functions
     * | Class is abstract and must be inherited and implemented due to expected implmentations (e.g. failsafe / write)
     * | Functionality is not constrained. It is up to the user to implement the necessary functions
     * | An inherited class can be found in /dronev2: ESC.h
     */
    class ThrusterInterface
    {
    public:
        struct failsafe_system // Failsafe system struct as example
        {
        private:
            bool failsafe = false;

        public:
            virtual bool engage() // Enable failsafe
            {
                failsafe = true;
                return true;
            }

            virtual bool disengage() // Disable failsafe
            {
                failsafe = false;
                return true;
            }

            virtual bool status() // Get failsafe status
            {
                return failsafe;
            }

            virtual ~failsafe_system() = default; // Destructor if needed
        };

        virtual int arm() = 0;              // Thruster arm function (e.g. ESC arming, or Booster initialization)
        virtual int stop() = 0;             // Thruster stop function (e.g. ESC stop, or Booster shutdown)
        virtual int restart() = 0;          // Thruster restart function (e.g. ESC restart, or Booster re-initialization)
        virtual int max() = 0;              // Thruster maximum speed function
        virtual int min() = 0;              // Thruster minimum speed function
        virtual int write(float speed) = 0; // Thruster write function (e.g. ESC write, or Booster write)

        failsafe_system failsafe;
    };

    /*
     * Drone event system
     * | Events have a severity level (notice, caution, error, critical)
     * | Events can have a descriptive message describing the issue
     * | Events can havea  short source description (e.g. "IMU", "GPS", "Battery")
     * | Events can have a code for quick reference (future implementation, requires a table)
     * | Events can be composed into a readable, developer-friendly string
     */
    struct drone_event_t
    {
        int _code;
        int _severity;
        String _sector;
        String _message;

        enum severity_t // Severity types
        {
            notice = 0,
            caution = 1,
            error = 2,
            critical = 3
        };

        drone_event_t(severity_t severity_, String message_, String sector_) // Recommended constructor
        {
            _code = 0;
            _severity = severity_;
            _sector = sector_;
            _message = message_;
        }

        drone_event_t(severity_t severity_, String message_) // Abbreviated alternative constructor
        {
            _code = 0;
            _severity = severity_;
            _sector = "unavailable";
            _message = message_;
        }

        // GET functions
        severity_t severity()
        {
            return static_cast<severity_t>(_severity);
        }

        String message()
        {
            if (_message == "")
            {
                return "Unavailable";
            }
            return _message;
        }

        String sector()
        {
            return _sector;
        }

        int code()
        {
            return _code;
        }

        String compose()
        {
            if (_sector != "unavailable")
            {
                return "[" + severityToString(_severity) + "] " + _sector + ": " + _message;
            }
            return "[" + severityToString(_severity) + "] " + _message;
        }

    private:
        // Helper function to map severity to string
        String severityToString(int severity_)
        {
            switch (severity_)
            {
            case 0:
                return "Notice";
                break;
            case 1:
                return "Caution";
                break;
            case 2:
                return "Error";
                break;
            case 3:
                return "Critical";
                break;
            default:
                return "Unknown";
                break;
            }
        }
    };

    /**
     * Drone class (Main API resides here)
     * | The main class for the drone API operations
     * | The class is templated to allow for pre-defined sensor "importing". The system therefore knows which sensors are available.
     * | This allows users to have a more controlled environment, and prevents runtime-issues.
     * |
     * | The class is designed to be a main point of access for all sub-systems, and no other module relies on this class (rule no. 1)
     * |
     * | Advanced sensor systems:
     * |    addSensor<Sensor>(args...) => Add a sensor of an imported type.
     * |    | Example: addSensor<IMU>(gyro, accel, mag);
     * |    |
     * |    sensor<Sensor>() => Get a sensor of an initialized type. (returns a static object)
     * |    | Example: sensor<IMU>().read();
     * |    |
     * |    [TIP]: Future implementations will alow for access of multiple sensors of the same type using a syntax as such:
     * |    | Example: sensor<IMU>(1).read(); // Read from IMU 1
     * |    | Example: sensor<IMU>(2).read(); // Read from IMU 2
     * |
     * | Flags (can be set by anyone):
     * |   profile_configured => Check if a profile is configured
     * |   thrusters_configured => Check if thrusters are configured (to prevent unnecessary action in functions like write or arm)
     * |
     * | Profile accessor: profile => Access the profile object as a pointer
     * |
     * | Drone frame: pose => Access the pose object for orientation and acceleration data (regular object) to keep orientation and acceleration data centralized
     * |
     * | Constructors:
     * |    Basic constructor: Drone() => For quick setup
     * |    Advanced constructor: Drone(Profile *drone_model) => For users who want to define parameters in a Profile object
     * |
     * | Advanced thruster systems:
     * |    setThrustControllers<Tc>({controllers...}) => Set thruster controllers (e.g. ESCs, Boosters)
     * |    | Example: setThrustControllers<ESC>({esc1, esc2, esc3, esc4});
     * |    |
     * |    arm() => Arm all thrusters
     * |    | Must be expanded in the future to allow for custom error logic from thruster interfaces (possibly a lambda in ThrusterInterface)
     * |    |
     * |    setMotorSpeeds(matrix motor_speeds_) => Set motor speeds using a matrix (rows = motors, columns = 1)
     * |    | Example: setMotorSpeeds({{1100}, {1100}, {1100}, {1100}});
     * |    [TIP]: Should only be used for direct matrix input from calculations. For a more user-friendly approach, use the next function.
     * |    |
     * |    setMotorSpeeds(speed1, speed2, speed3, speed4) => Set motor speeds using a list of speeds (argument count must match pre-defined thrusters)
     * |    | Example: setMotorSpeeds(1100, 1100, 1100, 1100);
     * |    [TIP]: This function is more user-friendly and allows for direct input of speeds without the need for a matrix.
     * |
     * | Future implementations:
     * |    - Proper Event system use and handling (drone_event_t)
     * |    - Sensor system expansion (e.g. multiple sensors of the same type)
     * |    - Advanced feedback support (e.g. sensor or motor feedback in case of failure)
     * |    - Strive towards more built-in safety features (e.g. failsafe system, aircraft pitch limits)
     * |    - Strive towards more built-in control features such as flight control systems (based on pose data)
     */
    template <typename... Sensors>
    class Drone
    {
    private:
        // Thruster storage vector
        std::vector<ThrusterInterface *> _thrusters = {};

        // Sensor storage vector (variant for multiple sensor types)
        struct DummySensor {}; // Dumym sensor, in case an instantiation like Drone<> is used.
        using sensorVariant = std::variant<DummySensor, Sensors...>;
        std::vector<sensorVariant> _sensors = {};

    public:
        // Flags for setup
        struct flags
        {
            bool profile_configured = false;
            bool thrusters_configured = false;
        } flags;

        // Profile accessor
        Profile *profile;

        // Drone orientation and acceleration data centralized
        struct Pose
        {
        private:
            int16_t *_gx, *_gy, *_gz = nullptr;
            float *_ax, *_ay, *_az = nullptr;
            float *_roll, *_pitch, *_yaw = nullptr;

        public:
            Pose &bindGx(int16_t *gx_)
            {
                _gx = gx_;
                return *this;
            }

            Pose &bindGy(int16_t *gy_)
            {
                _gy = gy_;
                return *this;
            }

            Pose &bindGz(int16_t *gz_)
            {
                _gz = gz_;
                return *this;
            }

            Pose &bindAx(float *ax_)
            {
                _ax = ax_;
                return *this;
            }

            Pose &bindAy(float *ay_)
            {
                _ay = ay_;
                return *this;
            }

            Pose &bindAz(float *az_)
            {
                _az = az_;
                return *this;
            }

            Pose &bindRoll(float *roll_)
            {
                _roll = roll_;
                return *this;
            }

            Pose &bindPitch(float *pitch_)
            {
                _pitch = pitch_;
                return *this;
            }

            Pose &bindYaw(float *yaw_)
            {
                _yaw = yaw_;
                return *this;
            }

            int16_t gx()
            {
                return *_gx;
            }

            int16_t gy()
            {
                return *_gy;
            }

            int16_t gz()
            {
                return *_gz;
            }

            float ax()
            {
                return *_ax;
            }

            float ay()
            {
                return *_ay;
            }

            float az()
            {
                return *_az;
            }

            float roll()
            {
                return *_roll;
            }

            float pitch()
            {
                return *_pitch;
            }

            float yaw()
            {
                return *_yaw;
            }
        } pose; // Pose accessor

        // Basic constructor
        Drone() {}

        // Advanced constructor
        Drone(Profile *drone_model)
        {
            profile = drone_model;
            flags.profile_configured = true;
        }

        // Add sensor function
        // Accepts a template of sensor type and argument list (for sensor constructor initialization)
        // Templates allow for initialization with type-safety, and has a syntax like:
        // function<type>(arguments...) where the type can be given before the logic, and arguments can directly be passed to the constructor.
        // Any failures then get caught during compilation.
        template <typename Sensor, typename... Args>
        Drone<Sensors...> &addSensor(Args &&...args) // Here we request a reference to the arguments to avoid copying (&&)
        {
            // Place the new sensor in the storage vector
            // Because the whole class is a template, the type argument can be reused here to accept of that type in the variant.
            // Make sure to forward the arguments to the constructor to avoid copying.
            // Adding ... after a series argument type (Args) unfolds the arguments into the function call.
            // such a template for series of arguments is called a variadic template.
            _sensors.emplace_back(Sensor(std::forward<Args>(args)...));

            // Return Drone for chaining
            return *this;
        }

        // Get sensor function
        // Just like before, we used a template for a requested type (St = Sensortype)
        // This function will return a reference to the sensor object of the requested type.
        // We loop thorugh the storage vector and "visit" the variant to check if the requested type is available. (std::holds_alternative)
        // Finally we return the sensor object using std::get which also uses a template to get a specific type from a vector.
        template <typename St>
        St &sensor()
        {
            for (auto &sensor : _sensors)
            {
                if (std::holds_alternative<St>(sensor))
                {
                    return std::get<St>(sensor);
                }
            }

            // (future) If the sensor is not found, we throw an exception.
        }

        // Set thruster controllers
        // Again, we use a type so we know which kind of controllers we are dealign with.
        // We use an initializer list to accept a list of controllers.
        // We clear the current thruster list and add the new controllers to the list to prevent duplicates.
        // We add the controllers using the new keyword (to create a pointer) to keep polymorphism.
        template <typename Tc>
        Drone<Sensors...> &setThrustControllers(std::initializer_list<Tc> controllers)
        {
            _thrusters.clear();
            for (const auto &controller : controllers)
            {
                _thrusters.push_back(new Tc(controller));
            }

            // Set the flag to true
            flags.thrusters_configured = true;

            // Return Drone for chaining
            return *this;
        }

        // Arm all thrusters
        // Failure handling is described respectively.
        int arm()
        {
            for (int i = 0; i < _thrusters.size(); i++)
            {
                int arm_status = _thrusters[i]->arm();
                if (arm_status == 1) // One arm has failed
                {
                    // [Set failsafe here with status code]
                    _thrusters[i]->failsafe.engage();
                    return 1; // Fail the whole thing for now.

                    // Future potential: return 2; // Partial failure
                    // Or a vector of status codes. This will allow for more detailed error handling.
                }
            }
            return 0;
        }

        // Apply motor speeds (write) using a matrix
        int setMotorSpeeds(matrix motor_speeds_)
        {
            if (motor_speeds_.rows() == _thrusters.size() && motor_speeds_.columns() == 1) // Rows = motors, Columns = 1
            {
                for (int i = 0; i < motor_speeds_.rows(); i++)
                {
                    _thrusters[i]->write(motor_speeds_(i + 1, 1));
                }
                return 0;
            }
            else
            {
                return 1;
            }
        }

        // Apply motor speeds using a list of speeds (variadic template)
        template <typename... MotorSpeeds>
        int setMotorSpeeds(MotorSpeeds... motor_speeds_)
        {
            std::vector<float> speeds = {motor_speeds_...};
            if (speeds.size() == _thrusters.size())
            {
                for (int i = 0; i < _thrusters.size(); i++)
                {
                    _thrusters[i]->write(speeds[i]);
                }
                return 0;
            }
            else
            {
                return 1;
            }
        }
    };
}

#endif
