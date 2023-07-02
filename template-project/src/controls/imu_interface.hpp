#ifndef IMU_INTERFACE_HPP_
#define IMU_INTERFACE_HPP_
#include "drivers.hpp"
#include "modm/math/geometry/angle.hpp"

#ifdef TARGET_STANDARD
#include "controls/standard/standard_constants.hpp"
#endif

#ifdef TARGET_SENTRY
#include "controls/sentry/sentry_constants.hpp"
#endif

#ifdef TARGET_HERO
#include "controls/hero/hero_constants.hpp"
#endif

/*
ImuRadInterfrace is a class that will return the values of imu readings in radians. It will also
use displacement angles when measuring position relative to its starting position.
due to taproot, yaw = roll, roll = pitch, pitch = yaw
*/

namespace tap{
    class Drivers;
}

class ImuRadInterface{
public:
    ImuRadInterface(tap::Drivers *drivers) : drivers(drivers) {}

    //getters for offset from angles from starting position from radians
    float getYaw() const{return modm::toRadian(drivers->mpu6500.getYaw()) + startingYaw;}
    float getPitch() const{return modm::toRadian(drivers->mpu6500.getPitch()) + startingPitch;}
    float getRoll() const{return modm::toRadian(drivers->mpu6500.getRoll()) + startingRoll;}

    //getters for velocities from gyro sensor, converted in rad/sec
    float getGyroX() const {return modm::toRadian(drivers->mpu6500.getGx());}
    float getGyroY() const {return modm::toRadian(drivers->mpu6500.getGy());}
    float getGyroZ() const {return modm::toRadian(drivers->mpu6500.getGz());}

    //getters for angular velocities in rad/sec^2
    float getAX() const {return modm::toRadian(drivers->mpu6500.getAx());}
    float getAY() const {return modm::toRadian(drivers->mpu6500.getAy());}
    float getAZ() const {return modm::toRadian(drivers->mpu6500.getAz());}

    bool ready() const {
        return tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATED ==
        drivers->mpu6500.getImuState();}

private:
    tap::Drivers *drivers;
    GIMBAL_CONSTANTS constants;

    //starting positions of yaw and pitch gimbal angles in radians.
    float startingPitch = constants.LEVEL_ANGLE + constants.PITCH_STARTING_ANGLE;
    float startingYaw = 0;
    float startingRoll = 0;

}; //class ImuInterface

#endif