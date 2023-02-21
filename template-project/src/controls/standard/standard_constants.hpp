#ifndef STANDARD_CONSTANTS_HPP_
#define STANDARD_CONSTANTS_HPP_

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
static constexpr float USER_JOYSTICK_PITCH_SCALAR = 0.15f;

static constexpr float CHASSIS_MOTOR_DISTANCE = 0.2f;
static constexpr float CHASSIS_ROTATION_SET_SCALE = 0.1f;

static constexpr float WHEELBASE_LENGTH = 0.366f;

static constexpr float LEVEL_ANGLE = 1.5708f; //90 degrees

static constexpr float YAW_ENCODER_OFFSET = 0.548f; //31.4 degrees

struct CHASSIS_CONSTANTS{
    //max output for chassis motors
    static constexpr float MAX_CURRENT_OUTPUT = 8000.0f;
    // Scale factor for converting joystick movement into RPM setpoint
    static constexpr float RPM_SCALE_FACTOR = 4000.0f;

    ///< Hardware constants, not specific to any particular chassis.
    static constexpr tap::motor::MotorId FRONT_LEFT_MOTOR_ID = tap::motor::MOTOR3;
    static constexpr tap::motor::MotorId FRONT_RIGHT_MOTOR_ID = tap::motor::MOTOR4;
    static constexpr tap::motor::MotorId BACK_RIGHT_MOTOR_ID = tap::motor::MOTOR1;
    static constexpr tap::motor::MotorId BACK_LEFT_MOTOR_ID = tap::motor::MOTOR2;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

    //M3505 motor speed PID 
    static constexpr float
    CHASSIS_MOTOR_KP = 20.0f,
    CHASSIS_MOTOR_KI = 0.2f,
    CHASSIS_MOTOR_KD = 0.0f,
    CHASSIS_MOTOR_MAX_IOUT = 2000.0f, //max integral 
    CHASSIS_MOTOR_MAX_OUT = 16000.0f; //max output
    
}; //struct CHASSIS_CONSTANTS


struct GIMBAL_CONSTANTS{
    //constants for YAW and PITCH Motor IDs
    static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;
    static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR6;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;

//Pid configs for gimbal Pid
    static constexpr tap::algorithms::SmoothPidConfig YAW_PID = {
        .kp = 600.0f,
        .ki = 0.0f,
        .kd = 500.0f,
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
        .kp = 550.0f, //1850.0f
        .ki = 0.0f,
        .kd = 150.0f,
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
static constexpr float YAW_SCALE = 0.25f;
static constexpr float PITCH_SCALE = 0.01f; // 0.0075f
//Gimbal Starting angles
static constexpr float YAW_STARTING_ANGLE = 0.0f;
static constexpr float PITCH_STARTING_ANGLE = 1.57079632679489661923f; //pi / 2
//Pitch Angle Limits
static constexpr float PITCH_MIN_ANGLE = 0.698132f; //40 degrees
static constexpr float PITCH_MAX_ANGLE = 2.61799; //150 degrees
//gimbal yaw and pitch speed limits
static constexpr float MIN_YAW_SPEED = 300.0f;
static constexpr float MAX_YAW_SPEED = 8000.0f; 
static constexpr float MIN_PITCH_SPEED = 300.0f;
static constexpr float MAX_PITCH_SPEED = 20000.0f;
//Gimbal minimum angles of movement
static constexpr float YAW_MINIMUM_RADS = .005f;
static constexpr float PITCH_MINIMUM_RADS = .0001f;
//minimum value for pitch RPM to be considered stable
static constexpr float MIN_PITCH_RPM = .0005f;
//starting pitch angle from when the robot is turned on 
static constexpr float  STARTING_PITCH = -0.855211f; //-49 degrees
//starting YAW Motor rotations
uint64_t STARTING_YAW_ROT = 10000000; // ten million, cause why not
//maximum pitch error
static constexpr float MAX_YAW_ERROR = M_PI / 2; //180 degrees

//values for gravity compensation
static constexpr float LEVEL_ANGLE = 1.5708; //90 degrees
static constexpr float BARREL_LENGTH = 165.0f; //turret barrel length in mm
static constexpr float BARREL_MIN_HEIGHT = 135.6f; 
static constexpr float BARREL_LEVEL_HEIGHT = 172.8f; 
static constexpr float GRAVITY_COMPENSATION_SCALAR = 5800;
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