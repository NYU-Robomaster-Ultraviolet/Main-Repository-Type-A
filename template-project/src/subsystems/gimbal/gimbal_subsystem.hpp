#ifndef GIMBAL_SUBSYSTEM_HPP_
#define GIMBAL_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"

#if defined (TARGET_STANDARD)
#include "controls/standard/standard_constants.hpp"
#elif defined (TARGET_HERO)
#include "controls/hero/hero_constants.hpp"
#elif defined (TARGET_SENTRY)
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
    //initializes all motors and sets base values for variables
    void initialize() override;

    /**
     * @brief loop that checks for changes in the target angle with current angle
     * and corrects until current angle is withing range of the target angle
     * 
     * @return ** void 
     */
    void refresh() override;

    const char* getName() override {return "gimbal subsystem";}

    //takes in a wrapped encoder value and turns it into radians
    static inline float wrappedEncoderValueToRadians(int64_t encoderValue);

    //wraps an angle around 0 and 2PI (0 - 360 degrees)
    float wrapAngle(float angle) const{
        if(angle > M_TWOPI) angle -= M_TWOPI;
        else if(angle < 0) angle += M_TWOPI;
        return angle;
    }

    //sets target yaw to wrapped angle
    void setYawAngle(float angle) {
        targetYaw = wrapAngle(angle);
    }

    //sets pitch angle, limits it between mechanical limits and adjusts to encoder offset
    void setPitchAngle(float angle) {targetPitch = limitVal<float>(angle , constants.PITCH_MIN_ANGLE + pitchEncoderOffset ,
        constants.PITCH_MAX_ANGLE + pitchEncoderOffset);}

    //sets encoder values, wrapping the yaw angle
    void setEncoderYawAngle(float angle) {
        encoderYaw = wrapAngle(angle);
    }
    void setEncoderPitchAngle(float angle) {
        encoderPitch = angle;
    }

    //returns the pitch given by the imu
    float getImuPitch(){ 
        // float pitch = modm::toRadian(drivers->mpu6500.getPitch());
        // float roll = modm::toRadian(drivers->mpu6500.getRoll());
        // tap::algorithms::rotateVector(&pitch, &roll, getImuYaw() - M_PI);
        //return pitch;
        
        float tilt = modm::toRadian(drivers->mpu6500.getTiltAngle());
        if(encoderPitch < LEVEL_ANGLE) tilt = -tilt;
        return tilt + LEVEL_ANGLE;
    }

    //returns yaw given by imu
    float getImuYaw() const {
        float yaw = -modm::toRadian(drivers->mpu6500.getYaw() - 180);
        return wrapAngle(yaw);
    }

    //sets stored imu angles and velocity calculations from time measurements
    void setYawImu(){ imuPitch = getImuYaw();}
    void setPitchImu(){ imuYaw = getImuPitch();}
    void findVelocityImu(uint32_t time);

    //get measured rpms of both motors
    float getYawMotorRPM() const {return yawMotor.isMotorOnline() ? yawMotor.getShaftRPM() : 0.0f; }
    float getPitchMotorRPM() const {return pitchMotor.isMotorOnline() ? pitchMotor.getShaftRPM() : 0.0f; }

    //getters for motor speeds in rad/s, rpm * (pi / 120)
    float getYawVelocity() const {return -(M_PI / 120) * yawMotor.getShaftRPM();}
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

    //checks if IMU is calibrated
    bool imuStatesCalibrated() const {return drivers->mpu6500.getImuState() == tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATED;}

    //alligns gimbal to chassis if not done already, else puts pitch to level position
    void allignGimbal();
    
    /**
     * @brief Finds the shortest rotation between two points, returning the target angle to achieve
     * said rotation
     * @param destination : the desired angle to set the motor to
     * @param yaw : if the motor is the yaw or pitch, true for yaw 
     * @return ** float 
     */
    float findRotation(float destination, bool yaw) const;

    //sets encoder offset by measuring the given encoder value against a measured angle
    void calibratePitch();

    //sets the beyblade mode gimbal compesation 0: no comp 1: counter clocksise, 2: clockwise, 3: half speed beyblading, 4: adjustment command
    void setBeybladeMode(unsigned char mode) {beybladeMode = mode;}

    //sets robot level, used for calculating beyblade offset
    void setRobotLevel(uint8_t lev) {level = lev;}

    //applies the beybladeOffset to the target angle
    void applyBeybladeOffset();

    //getter for the chassis beyblade speed
    float getChassisBeybladeSpeed() const {return beybladeChassisInput;}
 
    void changeCVMode(bool on) {cvActive = on;}

    void changeSlowBeyblade(float factor) {slowBeyblade = factor;}

    float getSlowBeyblade() const {return slowBeyblade;}

private:
    //motor interfaces
    tap::motor::DjiMotor yawMotor;
    tap::motor::DjiMotor pitchMotor;
    #if defined (TARGET_SENTRY) || defined (TARGET_HERO)
    tap::motor::DjiMotor pitchMotorL;
    #endif

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

    //check if alligned or not
    bool alligned = false;

    //things used for pitch angle calculations
    float pitchEncoderOffset = PITCH_ENCODER_OFFSET;
    bool calibratedPitch = false;

    //used for beyblade
    unsigned char beybladeMode = 0;

    //beyblade input scaler
    float beybladeGimbalInput = GIMBAL_BEYBLADE_INPUT;

    //updates the level of the chassis subsystem
    float beybladeChassisInput = BEYBLADE_INPUT;

    //gimbal level
    uint8_t level = 1;

    //cv is active or not
    bool cvActive;

    //beyblade at half speed or not
    float slowBeyblade = 1;

}; //class GimbalSubsystem
}//namespace gimbal


#endif