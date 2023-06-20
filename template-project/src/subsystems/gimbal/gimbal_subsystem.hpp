#ifndef GIMBAL_SUBSYSTEM_HPP_
#define GIMBAL_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"

#ifdef TARGET_STANDARD
#include "controls/standard/standard_constants.hpp"
#endif

#ifdef TARGET_HERO
#include "controls/hero/hero_constants.hpp"
#endif

#ifdef TARGET_SENTRY
#include "controls/sentry/sentry_constants.hpp"
#endif

#include "tap/util_macros.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "drivers.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/clock.hpp"
#include "controls/imu_interface.hpp"
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

    float wrapAngle(float angle) const{
        if(angle > M_TWOPI) angle -= M_TWOPI;
        else if(angle < 0) angle += M_TWOPI;
        return angle;
    }

    void setYawAngle(float angle) {
        targetYaw = wrapAngle(angle);
    }

    void setPitchAngle(float angle) {targetPitch = limitVal<float>(angle , constants.PITCH_MIN_ANGLE + PITCH_ENCODER_OFFSET ,
        constants.PITCH_MAX_ANGLE + PITCH_ENCODER_OFFSET);}

    void setEncoderYawAngle(float angle) {
        encoderYaw = wrapAngle(angle);
    }

    void setEncoderPitchAngle(float angle) {
        encoderPitch = angle;
    }

    void setPowerOutput(float yaw, float pitch){
        yawPowerOutput = yaw;
        pitchPowerOutput = pitch;
    }

    float getImuPitch(){ 
        // float pitch = modm::toRadian(drivers->mpu6500.getPitch());
        // float roll = modm::toRadian(drivers->mpu6500.getRoll());
        // tap::algorithms::rotateVector(&pitch, &roll, getImuYaw() - M_PI);
        //return pitch;
        
        float tilt = modm::toRadian(drivers->mpu6500.getTiltAngle());
        if(encoderPitch < LEVEL_ANGLE) tilt = -tilt;
        return tilt + LEVEL_ANGLE;
    }

    float getImuYaw() const {
        float yaw = -modm::toRadian(drivers->mpu6500.getYaw() - 180);
        return wrapAngle(yaw);
    }


    void setYawImu(){ imuPitch = getImuYaw();}
    void setPitchImu(){ imuYaw = getImuPitch();}
    void findVelocityImu(uint32_t time);


    float getYawMotorRPM() const {return yawMotor.isMotorOnline() ? yawMotor.getShaftRPM() : 0.0f; }
    float getPitchMotorRPM() const {return pitchMotor.isMotorOnline() ? pitchMotor.getShaftRPM() : 0.0f; }

    //getters for motor speeds in rad/s, rpm * (pi / 120)
    float getYawVelocity() const {return (M_PI / 120) * yawMotor.getShaftRPM();}
    float getPitchVelocity() const {return (M_PI / 120) * pitchMotor.getShaftRPM();}

    //getters for current motor positions
    float getYawEncoder() const {return encoderYaw;}
    float getPitchEncoder() const {return encoderPitch;}

    //getters for velocity
    float getImuVx() const {return imuVx;}
    float getImuVy() const {return imuVy;}
    float getImuVz() const {return imuVz;}

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

    //these are used for calibrating the IMU
    void calibrateImu() {calibrated = true;}
    bool isCalibrated() const {return calibrated;}

    bool imuStatesCalibrated() const {return drivers->mpu6500.getImuState() == tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATED;}

private:
    //motor interfaces
    tap::motor::DjiMotor yawMotor;
    tap::motor::DjiMotor pitchMotor;

    //starting angles
    float startingPitch;
    float startingYaw;
    //current angles in radians used for error calculations
    float targetYaw;
    float targetPitch;
    //current angles in radians from IMU
    float imuYaw = 0;
    float imuPitch = 0;
    //imu acceleration
    float imuAx = 0;
    float imuAy = 0;
    float imuAz = 0;
    //imu velocities
    float imuVx = 0;
    float imuVy = 0;
    float imuVz = 0;
    //current angles in radians from encoder
    float encoderYaw;
    float encoderPitch;
    //motor speed given in revolutions / min
    float currentYawMotorSpeed;
    float currentPitchMotorSpeed;

    //power output in percentages
    float yawPowerOutput;
    float pitchPowerOutput;

    //pid calculators that take in angular displacement and  angular velocity
    tap::algorithms::SmoothPid yawMotorPid;
    tap::algorithms::SmoothPid pitchMotorPid;

    //current angles in radians
    float currentYaw;
    float currentPitch;

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
    //checks if there are inputs or not
    bool inputsFound = false;

    //checks if gimbal imu is calibrated or not
    bool calibrated = false;

    //used to show that this is the first time setting imu
    bool firstSetYaw = true;
    bool firstSetPitch = true;

}; //class GimbalSubsystem
}//namespace gimbal


#endif