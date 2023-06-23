#include "cv_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"

namespace gimbal
{
CvCommand::CvCommand(GimbalSubsystem *const gimbal, src::Drivers *drivers)
: gimbal(gimbal), drivers(drivers)
{
     if (gimbal == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(gimbal));
}
void  CvCommand::initialize() {
    //gimbal->cvInput(findRotation(YAW_ENCODER_OFFSET), LEVEL_ANGLE - gimbal->getPitchEncoder());
    drivers->cv_com.changeCV(1);
}

void  CvCommand::execute() {
    //drivers->cv_com.setEncoder(gimbal->getYawEncoder(), gimbal->getPitchEncoder());
    drivers->cv_com.setImu(gimbal->getImuVx(), gimbal->getImuVy(), gimbal->getImuVz());
    drivers->cv_com.setAngles(gimbal->getYawEncoder(), gimbal->getPitchEncoder());

    if(drivers->cv_com.validReading()){
        gimbal->cvInput(drivers->cv_com.getYaw(), drivers->cv_com.getPitch());
        drivers->cv_com.invalidateAngle();
    }
    if(drivers->cv_com.getGimbalReadFlag()){
        float xInput = drivers->cv_com.getGimbalX();
        float yInput = drivers->cv_com.getGimbalY();
        gimbal->controllerInput(xInput, yInput);
        drivers->cv_com.resetGimbalReadFlag();
    }
    // if(drivers->cv_com.getGimbalPowerFlag()){
    //     float xInput = drivers->cv_com.getYawPower();
    //     float yInput = drivers->cv_com.getPitchPower();
    //     gimbal->controllerInput(xInput, yInput);
    //     drivers->cv_com.resetGimbalPowerFlag();
    // }
}

void  CvCommand::end(bool) {
    drivers->cv_com.changeCV(0);
    gimbal->cvInput(0, 0);
}

bool  CvCommand::isFinished() const { return false; }

float CvCommand::findRotation(const float& destination) const {
    float rotation = destination - gimbal->getYawEncoder();
    if(rotation > M_TWOPI) rotation = -(rotation - M_TWOPI);
    else if(rotation < -M_TWOPI) rotation = -(rotation + M_TWOPI);
    return rotation;
}


}//namespace cv