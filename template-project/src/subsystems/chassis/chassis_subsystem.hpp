#ifndef CHASSIS_SUBSYSTEM_HPP_
#define CHASSIS_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "subsystems/gimbal/gimbal_motor_interface.hpp"

#ifdef TARGET_STANDARD
#include "controls/standard/standard_constants.hpp"
#endif

#ifdef TARGET_HERO
#include "controls/hero/hero_constants.hpp"
#endif

#ifdef TARGET_SENTRY
#include "controls/sentry/sentry_constants.hpp"
#endif

#include "drivers.hpp"

namespace chassis
{
/**
 * A bare bones Subsystem for interacting with a 4 wheeled chassis.
 */

class ChassisSubsystem : public tap::control::Subsystem
{
public:
    /**
     * Constructs a new ChassisSubsystem with default parameters specified in
     * the private section of this class.
     */
    ChassisSubsystem(src::Drivers *drivers, gimbal::GimbalInterface * gimbal);

    /**
     * @brief Updates all information obtained from wheels
     * 
     * This information includes, encoder position, rpm, angular velocity, velocity in chassis frame, and velocity
     * in gimbal frame.
     */
    void updateWheelvalues();

    /**
     * @brief returns a pair containing the x and y components of velocity from a 2d rotation
     * NOTE: we are swapping x and y, because that's what the formula online uses
     * @param fowardVelo original X velocity (foward and backward)
     * @param rightVelo original Y velocity (left and right)
     * @param Offset the rotation used
     * @return ** std::pair<float, float> y , x
     */
    std::pair<float, float> transformVelocity(float fowardVelo, float rightVelo, float fowardOffset) const;

    //copy controls
    ChassisSubsystem(const ChassisSubsystem &other) = delete;

    ChassisSubsystem &operator=(const ChassisSubsystem &other) = delete;

    //default deconstructor
    ~ChassisSubsystem() = default;

    /**
     * Called when Subsystem is registered
     * initializes motors
     */
    void initialize() override;

    /**
     * Called continously as long as robot is running
     * Updates the Pid calculator with desired outputs
     */
    void refresh() override;

    /**
     * @param x : right and left inputs from controller
     * @param y : foward and backward inputs from controller
     * @param r : rotation from left or right inputs from controller
     * calculate the desired rpm of each wheel to have the desired movement
     */
    void setDesiredOutput(float x, float y, float r);

    /**
     * @brief Set the Velocity Output of motors
     * 
     * All values are already stored in the class
     * @return ** void 
     */
    void setVelocityOutput();

    /**
     * @brief Set the Target Velocity for the robot in the frame of view of the gimbal
     * 
     * @param x foward
     * @param y right
     * @param r rotation
     */
    void setTargetVelocity(float x, float y);
     void setRotationVelocity(float r);
    /**
     * @param pid : the pid calculator for the given motor
     * @param motor : the motor that needs to be updated
     * @param desiredRpm : the desired rpm set for tht motor
     * using pid calculator to calculate a rpm to set to given motor to after feeding the calculator
     * the desiredRpm
     */

    void updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRpm);

    //checks if every motor is online
    bool motorOnline() {return frontLeftMotor.isMotorOnline() && frontRightMotor.isMotorOnline() &&
    backLeftMotor.isMotorOnline() && backRightMotor.isMotorOnline();}

    //converts the motor absolute encoder value to radians
    inline float wrappedEncoderValueToRadians(int64_t encoderValue) {
        return (M_TWOPI * static_cast<float>(encoderValue)) / tap::motor::DjiMotor::ENC_RESOLUTION;
    }

    void setInputFlag(){inputFlag = 2;}

    //getters for each motor
    const tap::motor::DjiMotor &getFrontLeftMotor() const { return frontLeftMotor; }
    const tap::motor::DjiMotor &getFrontRightMotor() const { return frontRightMotor; }
    const tap::motor::DjiMotor &getBackLeftMotor() const { return backLeftMotor; }
    const tap::motor::DjiMotor &getBackRightMotor() const { return backRightMotor; }

private:
    //all constants used in this subsystem
    CHASSIS_CONSTANTS constants;

    ///< Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor frontLeftMotor;
    tap::motor::DjiMotor frontRightMotor;
    tap::motor::DjiMotor backLeftMotor;
    tap::motor::DjiMotor backRightMotor;

    // PID controllers for RPM feedback from wheels
    modm::Pid<float> frontLeftPid;
    modm::Pid<float> frontRightPid;
    modm::Pid<float> backLeftPid;
    modm::Pid<float> backRightPid;

    //flag for remote control style inputs
    char inputFlag;

    ///< Any user input is translated into desired RPM for each motor.
    float frontLeftDesiredRpm;
    float frontRightDesiredRpm;
    float backLeftDesiredRpm;
    float backRightDesiredRpm;


    //for caculating speeds from mecanum wheels
    gimbal::GimbalInterface * gimbalInterface;
    // FR = front right, FL = front left, BR = Back Right, BL = Back Left
    //Rotations per minute (degrees/min)
    float FRRPM = 0;
    float FLRPM = 0;
    float BRRPM = 0;
    float BLRPM = 0;
    //Encoder Position
    float FRPos = 0;
    float FLPos = 0;
    float BRPos = 0;
    float BLPos = 0;
    //Angular Velocity (w) rad/sec
    float FRVelocity = 0;
    float FLVelocity = 0;
    float BRVelocity = 0;
    float BLVelocity = 0;
    //robot reference values
    float longitudinalVelocity = 0; //foward and backward motion m/s
    float transversalVelocity = 0; //left and right motion m/s
    //rotational movement
    float angularVelocity = 0; //rad/s

    float directionOfMovement = 0; //rad
    float resultantVelocity = 0; //m/s

    //velocity in with the frame of view of the gimbal (wherever it points is forwards)
    float gimbalFrameVelocity = 0; //m/s
    float gimbalFrameVelocityX = 0; 
    float gimbalFrameVelocityY = 0;

    float targetFowardVelocity = 0;
    float targetRightVelocity = 0;
    float targetRotationVelocity = 0;

};  // class ChassisSubsystem

}  // namespace chassis


#endif  // CHASSIS_SUBSYSTEM_HPP_