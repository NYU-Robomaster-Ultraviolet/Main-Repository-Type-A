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
    gimbal->allignGimbal();
    drivers->cv_com.changeCV(true);
    gimbal->setBeybladeMode(0);
}

void  CvCommand::execute() {

    //update power limits
    // std::pair<uint16_t, uint16_t> powerLimit = drivers->ref_interface.getPowerUsage();
    // gimbal->setPower(powerLimit.first, powerLimit.second);

    // drivers->cv_com.setEncoder(gimbal->getYawEncoder(), gimbal->getPitchEncoder());
    // drivers->cv_com.setImu(gimbal->getImuVx(), gimbal->getImuVy(), gimbal->getImuVz());
    drivers->cv_com.setAngles(modm::toDegree(gimbal->getPitchEncoder() - PITCH_ENCODER_OFFSET) - 90, 
        modm::toDegree(gimbal->wrapAngle(gimbal->getYawEncoder() - YAW_ENCODER_OFFSET)));

    unsigned char beyblade = drivers->cv_com.getBeybladeMode();
    float yawInput = 0;
    float pitchInput = 0;
    gimbal->setBeybladeMode(drivers->cv_com.getBeybladeMode());
    if(drivers->cv_com.validReading()){
        yawInput = drivers->cv_com.getYaw();
        pitchInput = drivers->cv_com.getPitch();
        drivers->cv_com.invalidateAngle();
        gimbal->cvInput(yawInput, pitchInput);
    }
    // else if(drivers->cv_com.getGimbalReadFlag()){
    //     //float xInput = drivers->cv_com.getGimbalX();
    //     float yInput = drivers->cv_com.getGimbalY();
    //     gimbal->controllerInput(yInput, 0);
    //     drivers->cv_com.resetGimbalReadFlag();
    // }
    //beyblade inputs
}

void  CvCommand::end(bool) {
    gimbal->setBeybladeMode(0);
    drivers->cv_com.changeCV(0);
    gimbal->cvInput(0, 0);
    gimbal->noInputs();
}

bool  CvCommand::isFinished() const { return false; }



}//namespace cv