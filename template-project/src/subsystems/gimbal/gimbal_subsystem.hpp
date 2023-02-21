#ifndef GIMBAL_SUBSYSTEM_HPP_
#define GIMBAL_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "controls/standard/standard_constants.hpp"
#include "tap/util_macros.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "drivers.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/clock.hpp"
#include "controls/standard/imu_interface.hpp"
#include "modm/math/geometry/angle.hpp"

using namespace tap::algorithms;

namespace gimbal{
class GimbalSubsystem : public tap::control::Subsystem
{
public:
    GimbalSubsystem(src::Drivers *drivers);

    void initialize() override;
    void refresh() override;

    const char* getName() override {return "gimbal subsystem";}

    static inline float wrappedEncoderValueToRadians(int64_t encoderValue);

    void setYawAngle(float angle) { 
        if(angle > M_TWOPI) angle -= M_TWOPI;
        else if(angle < 0) angle += M_TWOPI;
        targetYaw = angle;
    }

    void setPitchAngle(float angle) {targetPitch = limitVal<float>(angle , constants.PITCH_MIN_ANGLE , 
        constants.PITCH_MAX_ANGLE);}

    float getYawMotorRPM() const {return yawMotor.isMotorOnline() ? yawMotor.getShaftRPM() : 0.0f; }
    float getPitchMotorRPM() const {return pitchMotor.isMotorOnline() ? pitchMotor.getShaftRPM() : 0.0f; }

    //getters for motor speeds in rad/s, rpm * (pi / 120)
    float getYawVelocity() const {return (M_PI / 120) * yawMotor.getShaftRPM();}
    float getPitchVelocity() const {return (M_PI / 120) * pitchMotor.getShaftRPM();}

    //getters for current motor positions
    float getYawEncoder() const {return currentYaw;}
    float getPitchEncoder() const {return currentPitch;}

    //these methods will update both PID calculators and set motor speeds
    void updateYawPid();
    void updatePitchPid();

    //checks to see if motors are online or not
    bool yawOnline() const {return yawMotor.isMotorOnline();}
    bool pitchOnline() const {return pitchMotor.isMotorOnline();}

    /*these methods cover the three posibilities of gimbal position:
    either controller inputs, CV inputs, or no inputs*/
    void controllerInput(float yawInput, float pitchInput);
    void cvInput(float yawInput, float pitchInput);

    //checks if there are no inputs
    void noInputs();

    //this methods will take into consideration the current pitch of the gimbal and return a float value that will lock it in place
    float gravityCompensation();

    //this method sets the IMU pitch angles
    void setIMU(float yaw, float p);

private:
    tap::motor::DjiMotor yawMotor;
    tap::motor::DjiMotor pitchMotor;

    //starting angle
    float startingPitchEncoder;
    float startingYawEncoder;

    float startingPitch;
    float startingYaw;
    //current angles in radians from motor encoder data
    float targetYaw;
    float targetPitch;
    //current angles in radians from IMU
    float imuYaw;
    float imuPitch;
    //motor speed given in revolutions / min
    float currentYawMotorSpeed;
    float currentPitchMotorSpeed;

    //pid calculators that take in angular displacement and  angular velocity
    tap::algorithms::SmoothPid yawMotorPid;
    tap::algorithms::SmoothPid pitchMotorPid;

    //current angles in radians
    float currentYaw;
    float currentPitch;

    //yaw flag to determine direction 1 = right 0 = left
    bool rightTurnFlag;
    //yaw flag to determine if there is no turning at the moment
    bool noTurn;

    //desired error angles
    float yawError;
    float pitchError;

    //last time measurement
    u_int32_t pastTime = 0;
    //difference in time
    u_int32_t timeError = 0;

    //desired output values for motors in current
    float yawMotorOutput;
    float pitchMotorOutput;
    //all other gimbal constants
    GIMBAL_CONSTANTS constants;
    //Gimbal PID output to motor speed error factor
    float motorSpeedFactor;
    //checks if there are inputs or not
    bool inputsFound = false;

    //imu interface
    ImuRadInterface imu;
}; //class GimbalSubsystem
}//namespace gimbal


#endif