#ifndef SENTRY_CONSTANTS_HPP_
#define SENTRY_CONSTANTS_HPP_
#ifdef TARGET_SENTRY

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

#include "modm/math/geometry/angle.hpp"

/**
 * @brief constants used for configeration of classes when building standard robot
 *
 */

//what we consider to be a level position for the gimbal
static constexpr float LEVEL_ANGLE = 1.5708f;  // 90 degrees

//offset to the angle that the encoder gives you
static constexpr float YAW_ENCODER_OFFSET = 4.98f;          // 165.6 degrees
static constexpr float PITCH_ENCODER_OFFSET = 3.05433;//175 //3.31613f; //190 //3.75246f;

//inputs used for chassis beyblading
static constexpr float BEYBLADE_INPUT = .7f;//.4f;
static constexpr float BEYBLADE_INPUT_TWO = .8f;//.4f
static constexpr float BEYBLADE_INPUT_THREE = .9f;//.4f
//input used for gimbal beyblading
static constexpr float GIMBAL_BEYBLADE_INPUT = -BEYBLADE_INPUT * .6f;
static constexpr float GIMBAL_BEYBLADE_ANGLE_INPUT = -BEYBLADE_INPUT * .645;
static constexpr float GIMBAL_BEYBLADE_INPUT_TWO = -BEYBLADE_INPUT_TWO * .6f;
static constexpr float GIMBAL_BEYBLADE_ANGLE_INPUT_TWO = -BEYBLADE_INPUT_TWO * .645;
static constexpr float GIMBAL_BEYBLADE_INPUT_THREE = -BEYBLADE_INPUT_THREE * .6f;
static constexpr float GIMBAL_BEYBLADE_ANGLE_INPUT_THREE = -BEYBLADE_INPUT_THREE * .645;

static constexpr float GIMBAL_BEYBLADE_CV_OFFSET_ONE = 1.1;

//speeds of flywheels at different levels
static constexpr float LEVEL_ONE_FLYWHEEL = .5f;
static constexpr float LEVEL_TWO_FLYWHEEL = .5f;
static constexpr float LEVEL_THREE_FLYWHEEL = .5f;

//speeds feeder at different levels
static constexpr float LEVEL_ONE_FEEDER_RPM = 1500;
static constexpr float LEVEL_TWO_FEEDER_RPM = 2000;
static constexpr float LEVEL_THREE_FEEDER_RPM = 2500;

//constants used for feeder motor subsystem
struct Feeder_CONSTANTS
{
    static constexpr tap::motor::MotorId FEEDER_MOTOR_ID = tap::motor::MOTOR7;
    static constexpr tap::motor::MotorId FEEDER_MOTOR_ID2 = tap::motor::MOTOR8;
    static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS2;
    static constexpr bool FEEDER_REVERSED = false;
    static constexpr bool FEEDER_REVERSED_2 = true;


};

//constants for feeder pid
struct FEEDER_PID
{
    static constexpr float PID_KP = 800.0f, PID_KI = 0.5f, PID_KD = 0.0f, PID_MAX_OUT = 10000.0f,
                           PID_MAX_IOUT = 9000.0f;
};  // struct FEEDER_PID

//constants used for chassis motor subsystem
struct CHASSIS_CONSTANTS
{
    //power limits at each level
    const float POWER_LIMIT_ONE = 8000;
    const float POWER_LIMIT_TWO = 8000;
    const float POWER_LIMIT_THREE = 8000;

    // max output for chassis motors
    static constexpr float MAX_CURRENT_OUTPUT = 8000.0f;
    // Scale factor for converting joystick movement into RPM setpoint
    static constexpr float RPM_SCALE_FACTOR = 4000.0f;

    ///< Hardware constants, not specific to any particular chassis.
    static constexpr tap::motor::MotorId FRONT_LEFT_MOTOR_ID = tap::motor::MOTOR3;
    static constexpr tap::motor::MotorId FRONT_RIGHT_MOTOR_ID = tap::motor::MOTOR4;
    static constexpr tap::motor::MotorId BACK_RIGHT_MOTOR_ID = tap::motor::MOTOR1;
    static constexpr tap::motor::MotorId BACK_LEFT_MOTOR_ID = tap::motor::MOTOR2;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

    static constexpr bool FRONT_LEFT_REVERSED = false;
    static constexpr bool FRONT_RIGHT_REVERSED = true;
    static constexpr bool BACK_RIGHT_REVERSED = true;
    static constexpr bool BACK_LEFT_REVERSED = false;

    // M3505 motor speed PID
    static constexpr float CHASSIS_MOTOR_KP = 20.0f, CHASSIS_MOTOR_KI = 0.2f,
                           CHASSIS_MOTOR_KD = 0.0f,
                           CHASSIS_MOTOR_MAX_IOUT = 2000.0f,  // max integral
        CHASSIS_MOTOR_MAX_OUT = 8000.0f;                      // max output 16000

    // values for calculating speeds from mecanum wheels

    // distance of wheels from center
    static constexpr float WHEEL_FROM_CENTER = 0.32f;  // meters
    // distance fron left to right wheels
    static constexpr float LEFT_TO_RIGHT_WHEEL = 0.54f;  // meters
    // distance from front to back wheels
    static constexpr float FRONT_TO_BACK_WHEEL = 0.34f;  // meters
    // number of wheels
    static constexpr unsigned char NUM_WHEELS = 4;
    // wheel roller radius
    static constexpr float ROLLER_RADIUS = .01f;  // meters
                                                  // constants for CV movement
    static constexpr float MIN_RADIANS = .01f;    // min radians to travel to
    static constexpr float MIN_DISTANCE = .01;    // min distance to travel in meters
};                                                // struct CHASSIS_CONSTANTS

//constants used for gimbal motor subsystem
struct GIMBAL_CONSTANTS
{
    // constants for YAW and PITCH Motor IDs
    static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR6;
    static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR5;
    static constexpr tap::motor::MotorId PITCH_LEFT_MOTOR_ID = tap::motor::MOTOR6;
    //which can line they are on (based off of which esc they are wired to)
    static constexpr tap::can::CanBus CAN_BUS_MOTORS_YAW = tap::can::CanBus::CAN_BUS1;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS_PITCH = tap::can::CanBus::CAN_BUS2;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS_PITCH_LEFT = tap::can::CanBus::CAN_BUS2;
    //if the motor direction is reversed (based on how the motors are mounted)
    static constexpr bool YAW_REVERSED = false;
    static constexpr bool PITCH_REVERSED = false;
    static constexpr bool PITCH_LEFT_REVERSED = true;

    static constexpr tap::algorithms::SmoothPidConfig YAW_PID = {
        .kp = 80000.0f,  // 60000
        .ki = 0,
        .kd = 920,  // 500
        .maxICumulative = 0.0f,
        .maxOutput = 32000.0f,
        .tQDerivativeKalman = 1.0f,
        .tRDerivativeKalman = 40.0f,
        .tQProportionalKalman = 1.0f,
        .tRProportionalKalman = 0.0f,
        .errDeadzone = 0.0f,
        .errorDerivativeFloor = 0.0f,
    };

    static constexpr tap::algorithms::SmoothPidConfig PITCH_PID = {
        .kp = 200.0f,  // 400
        .ki = 0.0f,
        .kd = 150.0f,  // 150
        .maxICumulative = 10.0f,
        .maxOutput = 16000.0f,
        .tQDerivativeKalman = 1.0f,
        .tRDerivativeKalman = 10.0f,
        .tQProportionalKalman = 1.0f,
        .tRProportionalKalman = 0.0f,
        .errDeadzone = 0.5f,
        .errorDerivativeFloor = 0.0f,
    };  // struct GIMBAL_SMOOTH_PID

    // Gimbal Motor values

    // Gimbal PID output to motor speed error factor
    static constexpr float MOTOR_SPEED_FACTOR = 100.0f;

    // the value in which controller inputs are multiplied by for gimbal movement, basically
    // sensitivity
    static constexpr float YAW_SCALE = 0.0125f;  //.25
    static constexpr float PITCH_SCALE = 0.01f;  // 0.0075f
    // Gimbal Starting angles
    static constexpr float YAW_STARTING_ANGLE = 0;
    static constexpr float PITCH_STARTING_ANGLE = 2.05949f;  // 118 degrees
    // Pitch Angle Limits
    static constexpr float PITCH_MIN_ANGLE = 1.309f; //75 degrees //1.22173f; //70
    static constexpr float PITCH_MAX_ANGLE = 1.91986f; //110 //2.05949f;  // 118 degrees, equal to starting
    static constexpr float PITCH_MECHANICAL_MIN_ANGLE = 1.13446f; // 65 degrees
    static constexpr float PITCH_MECHANICAL_MAX_ANGLE = 2.05949f; // 118 degrees
    // gimbal yaw and pitch speed limits
    static constexpr float MIN_YAW_SPEED = 20.0f;  // 300
    static constexpr float MAX_YAW_SPEED = 30000.0f;
    static constexpr float MIN_PITCH_SPEED = 100.0f; //20
    static constexpr float MAX_PITCH_SPEED = 30000.0f;  // 20000
    // Gimbal minimum angles of movement
    static constexpr float YAW_MINIMUM_RADS = 0.00436332;  //.25 //.5 degrees
    static constexpr float PITCH_MINIMUM_RADS = .0001f;
    static constexpr float YAW_MINIMUM_IMU_RADS = 0.0349066;  // 2 degrees
    // minimum value for pitch RPM to be considered stable
    static constexpr float MIN_PITCH_RPM = .0005f;
    // starting YAW Motor rotations
    uint64_t STARTING_YAW_ROT = 10000000;  // ten million, cause why not
    // maximum pitch error
    static constexpr float MAX_YAW_ERROR = M_PI;  // 180 degrees

    // values for gravity compensation
    static constexpr float LEVEL_ANGLE = 1.5708;    // 90 degrees
    static constexpr float BARREL_LENGTH = 165.0f;  // turret barrel length in mm
    static constexpr float BARREL_MIN_HEIGHT = 135.6f;
    static constexpr float BARREL_LEVEL_HEIGHT = 172.8f;
    static constexpr float GRAVITY_COMPENSATION_SCALAR = -5000;
};  // struct GIMBAL_CONSTANTS


#endif
#endif