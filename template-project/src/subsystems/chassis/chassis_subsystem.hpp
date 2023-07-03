#ifndef CHASSIS_SUBSYSTEM_HPP_
#define CHASSIS_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "subsystems/gimbal/gimbal_motor_interface.hpp"

#if defined (TARGET_STANDARD)
#include "controls/standard/standard_constants.hpp"
#elif defined (TARGET_HERO)
#include "controls/hero/hero_constants.hpp"
#elif defined (TARGET_SENTRY)
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
     * @brief Set the Target distance of movement for the robot in the frame of view of the gimbal
     * 
     * @param x foward
     * @param r rotation
     */
    void setRotationRadians(float r);
    void setFowardMovement(float x);

    //sets the beyblade mode, would limit linear movements to save power
    void setBeybladeMode(int a) {beybladeMode = a;}
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

    //lowers the speed limits
    void limitPower(float ratio);

    //sets flag to tell if it is in velocity movement mode true = yes
    void changeVelocityMoveFlag(bool flag){velocityMoveFlag = flag;}

    void moveAllignWithGimbal();

    //getters for each motor
    const tap::motor::DjiMotor &getFrontLeftMotor() const { return frontLeftMotor; }
    const tap::motor::DjiMotor &getFrontRightMotor() const { return frontRightMotor; }
    const tap::motor::DjiMotor &getBackLeftMotor() const { return backLeftMotor; }
    const tap::motor::DjiMotor &getBackRightMotor() const { return backRightMotor; }

    //getters for chassis speeds in x direction
    float getLongitude() const {return longitudinalVelocity;}
    float getGimbalFrameX() const {return gimbalFrameVelocityX;}

    //getters for chassis speeds in y direction
    float getTransversal() const {return transversalVelocity;}
    float getGimbalFrameY() const {return gimbalFrameVelocityY;}

    //getters for chassis rotational velocity
    float getRotationVelocity() const {return angularVelocity;}

    float getTargetRotation() const {return targetRadians;}

    float getTargetDistance() const {return targetDistance;}

    float getTargetVelocity() const {return targetFowardVelocity;}

    float getAcceleration() const {return longitudinalAcceleration;}

    float getDistanceX() const {return chassisFrameDistanceTraveledX;}

    float getFowardSampleFactor() const {return currFowardSampleInput;}

    float getRotationSampleFactor() const {return currRotationSampleInput;}

    float getTargetRotationalVelocity() const {return targetRotationVelocity;}

    //set beyblade mode and direction
    void setBeybladeMode(uint8_t mode) {beybladeMode = mode;}
private:
    //all constants used in this subsystem
    CHASSIS_CONSTANTS constants;


    //maximum power a robot wheel can use
    #if defined (TARGET_STANDARD)
    const float STARTING_POWER_LIMIT = 6000.0f;
    #elif defined (TARGET_SENTRY)
    const float STARTING_POWER_LIMIT = 8000.0f;
    #elif defined (TARGET_HERO)
    const float STARTING_POWER_LIMIT = 8000.0f;
    #endif
    float maximumPower = STARTING_POWER_LIMIT;
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

    //stored inputs
    float lastX = 0;
    float lastY = 0;
    float lastR = 0;

    //flag for remote control style inputs
    bool velocityMoveFlag = false;

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
    float prevLongitudinalVelocity = 0; //last read versions
    float prevTransversalVelocity = 0; //
    float longitudinalAcceleration = 0; //estimated accelerations
    float transversalAcceleration = 0;
    //rotational movement
    float angularVelocity = 0; //rad/s
    float prevAngularVelocity = 0;
    float angularAcceleration = 0;

    float directionOfMovement = 0; //rad
    float resultantVelocity = 0; //m/s

    //velocity in with the frame of view of the gimbal (wherever it points is forwards)
    float gimbalFrameVelocity = 0; //m/s
    float gimbalFrameVelocityX = 0; 
    float gimbalFrameVelocityY = 0;

    float targetFowardVelocity = 0;
    float targetRightVelocity = 0;
    float targetRotationVelocity = 0;
    float currFowardSampleInput = 0;
    float currRightSampleInput = 0;
    float currRotationSampleInput = 0;

    float targetRadians = 0;

    float radiansTraveled = 0;

    float targetDistance = 0; //in mm;

    bool stopFlag = false;
    float gimbalFrameDistanceTraveled = 0;
    float chassisFrameDistanceTraveledX = 0;
    float chassisFrameDistanceTraveledY = 0;

    uint32_t prevTime = 0; //in ms;
    uint32_t timeError = 0;

    //mode of beyblade, 0: off, 1-3 corolate with speeds given by level
    uint8_t beybladeMode = 0;

};  // class ChassisSubsystem

}  // namespace chassis


#endif  // CHASSIS_SUBSYSTEM_HPP_