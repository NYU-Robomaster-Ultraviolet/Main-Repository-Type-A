#ifndef SENTRY_CONSTANTS_HPP_
#define SENTRY_CONSTANTS_HPP_
#ifdef TARGET_SENTRY

#include "tap/algorithms/smooth_pid.hpp"
#include "modm/math/geometry/angle.hpp"
#include "tap/motor/dji_motor.hpp"

/**
 * @brief constants used for configeration of classes when building standard robot
 *
 */


static constexpr short int USER_MOUSE_YAW_MAX = 1000;
static constexpr short int USER_MOUSE_PITCH_MAX = 1000;
static constexpr float USER_MOUSE_YAW_SCALAR = (1.0f / USER_MOUSE_YAW_MAX);
static constexpr float USER_MOUSE_PITCH_SCALAR = (1.0f / USER_MOUSE_PITCH_MAX);

//static constexpr float USER_JOYSTICK_YAW_SCALAR = 0.3f;
//static constexpr float USER_JOYSTICK_PITCH_SCALAR = 0.15f;

static constexpr float USER_JOYSTICK_YAW_SCALAR = 0.3f;
static constexpr float USER_JOYSTICK_PITCH_SCALAR = 0.2f; //.15

static constexpr float CHASSIS_MOTOR_DISTANCE = 0.2f;
static constexpr float CHASSIS_ROTATION_SET_SCALE = 0.1f;

static constexpr float WHEELBASE_LENGTH = 0.366f;

static constexpr float LEVEL_ANGLE = 1.5708f; //90 degrees

static constexpr float YAW_ENCODER_OFFSET = 2.8902652; //165.6 degrees
static constexpr float PITCH_ENCODER_OFFSET = 3.780138814; //216.586 degrees

static constexpr float BEYBLADE_INPUT = .4f;

struct Feeder_CONSTANTS{
    static constexpr tap::motor::MotorId FEEDER_MOTOR_ID = tap::motor::MOTOR7;
    static constexpr tap::motor::MotorId FEEDER_MOTOR_ID2 = tap::motor::MOTOR8;
    static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS2;
}; 

struct CHASSIS_CONSTANTS{
    //max output for chassis motors
    static constexpr float MAX_CURRENT_OUTPUT = 8000.0f;
    // Scale factor for converting joystick movement into RPM setpoint
    static constexpr float RPM_SCALE_FACTOR = 4000.0f;

    ///< Hardware constants, not specific to any particular chassis.
    static constexpr tap::motor::MotorId FRONT_LEFT_MOTOR_ID = tap::motor::MOTOR1;
    static constexpr tap::motor::MotorId FRONT_RIGHT_MOTOR_ID = tap::motor::MOTOR2;
    static constexpr tap::motor::MotorId BACK_RIGHT_MOTOR_ID = tap::motor::MOTOR3;
    static constexpr tap::motor::MotorId BACK_LEFT_MOTOR_ID = tap::motor::MOTOR4;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

    //M3505 motor speed PID
    static constexpr float
    CHASSIS_MOTOR_KP = 20.0f,
    CHASSIS_MOTOR_KI = 0.2f,
    CHASSIS_MOTOR_KD = 0.0f,
    CHASSIS_MOTOR_MAX_IOUT = 2000.0f, //max integral
    CHASSIS_MOTOR_MAX_OUT = 8000.0f; //max output 16000

    //values for calculating speeds from mecanum wheels

    //distance of wheels from center
    static constexpr float WHEEL_FROM_CENTER = 0.32f; //meters
    //distance fron left to right wheels 
    static constexpr float LEFT_TO_RIGHT_WHEEL = 0.54f; //meters
    //distance from front to back wheels
    static constexpr float FRONT_TO_BACK_WHEEL = 0.34f; //meters
    //number of wheels
    static constexpr unsigned char NUM_WHEELS = 4;
    //wheel roller radius
    static constexpr float ROLLER_RADIUS = .01f; // meters
    
}; //struct CHASSIS_CONSTANTS


struct GIMBAL_CONSTANTS{
    //constants for YAW and PITCH Motor IDs
    static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR6;
    static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR5;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS_PITCH = tap::can::CanBus::CAN_BUS2;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS_YAW = tap::can::CanBus::CAN_BUS1;

//Pid configs for gimbal Pid
    static constexpr tap::algorithms::SmoothPidConfig YAW_PID = {
        .kp = 700.0f,
        .ki = 0.0f,
        .kd = 600.0f, //500
        .maxICumulative = 10.0f,
        .maxOutput = 16000.0f,
        .tQDerivativeKalman = 1.0f,
        .tRDerivativeKalman = 1.0f,
        .tQProportionalKalman = 1.0f,
        .tRProportionalKalman = 1.0f,
        .errDeadzone = 0.0f,
        .errorDerivativeFloor = 0.0f,
    };

    static constexpr tap::algorithms::SmoothPidConfig PITCH_PID = {
        .kp = 400.0f, //900.0
        .ki = 0.0f,
        .kd = 150.0f, //150
        .maxICumulative = 10.0f,
        .maxOutput = 16000.0f,
        .tQDerivativeKalman = 1.0f,
        .tRDerivativeKalman = 1.0f,
        .tQProportionalKalman = 1.0f,
        .tRProportionalKalman = 1.0f,
        .errDeadzone = 0.5f,
        .errorDerivativeFloor = 0.0f,
    }; //struct GIMBAL_SMOOTH_PID

//Gimbal Motor values


//Gimbal PID output to motor speed error factor
static constexpr float MOTOR_SPEED_FACTOR = 200.0f;

//the value in which controller inputs are multiplied by for gimbal movement, basically sensitivity
static constexpr float YAW_SCALE = 0.0125f; //.25
static constexpr float PITCH_SCALE = 0.01f; // 0.0075f
//Gimbal Starting angles
static constexpr float YAW_STARTING_ANGLE = 0.0f;
static constexpr float PITCH_STARTING_ANGLE = 1.91986f; //110 degrees
//Pitch Angle Limits
static constexpr float PITCH_MIN_ANGLE = 1.22173f; //70 degrees
static constexpr float PITCH_MAX_ANGLE = 1.91986f; //110 degrees, equal to starting
//gimbal yaw and pitch speed limits
static constexpr float MIN_YAW_SPEED = 300.0f;
static constexpr float MAX_YAW_SPEED = 8000.0f;
static constexpr float MIN_PITCH_SPEED = 300.0f;
static constexpr float MAX_PITCH_SPEED = 16000.0f; //20000
//Gimbal minimum angles of movement
static constexpr float YAW_MINIMUM_RADS = 0.0174533f; // 1 degree //0.0349066f; // 2 degrees
static constexpr float PITCH_MINIMUM_RADS = .0001f;
//minimum value for pitch RPM to be considered stable
static constexpr float MIN_PITCH_RPM = .0005f;
//starting pitch angle from when the robot is turned on
static constexpr float  STARTING_PITCH = -0.855211f; //-49 degrees
//starting YAW Motor rotations
uint64_t STARTING_YAW_ROT = 10000000; // ten million, cause why not
//maximum pitch error
static constexpr float MAX_YAW_ERROR = M_PI; //180 degrees

//values for gravity compensation
static constexpr float LEVEL_ANGLE = 1.5708; //90 degrees
static constexpr float BARREL_LENGTH = 165.0f; //turret barrel length in mm
static constexpr float BARREL_MIN_HEIGHT = 135.6f;
static constexpr float BARREL_LEVEL_HEIGHT = 172.8f;
static constexpr float GRAVITY_COMPENSATION_SCALAR = -5800;
};//struct GIMBAL_CONSTANTS

struct FEEDER_PID
{
    static constexpr float
    PID_KP = 800.0f,
    PID_KI = 0.5f,
    PID_KD = 0.0f,
    PID_MAX_OUT = 10000.0f,
    PID_MAX_IOUT = 9000.0f;
};  // struct FEEDER_PID

#endif
#endif