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
}

void  CvCommand::execute() {
    drivers->cv_com.setEncoder(gimbal->getYawEncoder(), gimbal->getPitchEncoder());
    drivers->cv_com.setImu(gimbal->getImuVx(), gimbal->getImuVy(), gimbal->getImuVz());
    drivers->cv_com.setAngles(gimbal->getYawEncoder(), gimbal->getPitchEncoder());
    unsigned char beyblade = drivers->cv_com.getBeybladeMode();
    float yawInput = 0;
    float pitchInput = 0;

    if(drivers->cv_com.validReading()){
        yawInput = drivers->cv_com.getYaw();
        pitchInput = drivers->cv_com.getPitch();
        drivers->cv_com.invalidateAngle();
    }
    // if(drivers->cv_com.getGimbalReadFlag()){
    //     float xInput = drivers->cv_com.getGimbalX();
    //     float yInput = drivers->cv_com.getGimbalY();
    //     gimbal->controllerInput(xInput, yInput);
    //     drivers->cv_com.resetGimbalReadFlag();
    // }
    //beyblade inputs
    // if(beyblade = 1)
    //         yawInput += (GIMBAL_BEYBLADE_ANGLE_INPUT); // ~22 degrees
    // else if(beyblade = 2)
    //     yawInput -= (GIMBAL_BEYBLADE_ANGLE_INPUT);

    gimbal->cvInput(yawInput, pitchInput);
}

void  CvCommand::end(bool) {
    drivers->cv_com.changeCV(0);
    gimbal->cvInput(0, 0);
    gimbal->noInputs();
}

bool  CvCommand::isFinished() const { return false; }



}//namespace cv