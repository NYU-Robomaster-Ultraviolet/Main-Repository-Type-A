#include "cv_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"

#ifdef TARGET_STANDARD
#include "controls/standard/standard_constants.hpp"
#endif

#ifdef TARGET_SENTRY
#include "controls/sentry/sentry_constants.hpp"
#endif


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
    gimbal->cvInput(findRotation(YAW_ENCODER_OFFSET), LEVEL_ANGLE - gimbal->getPitchEncoder());
}

void  CvCommand::execute() {

    if(drivers->cv_com.validReading()){
        gimbal->cvInput(drivers->cv_com.getYaw(), drivers->cv_com.getPitch());
        drivers->cv_com.invalidateAngle();
    }

    float yaw = gimbal->getImuYaw();
    float pitch = gimbal->getImuPitch();

    // 'c' is a random char used as header to fit the format
    std::string input = 'c' + std::to_string(yaw) + std::to_string(pitch);
    drivers->cv_com.writeToUart(input);
}

void  CvCommand::end(bool) {

    }

bool  CvCommand::isFinished() const { return false; }

float CvCommand::findRotation(const float& destination) const {
    float rotation = destination - gimbal->getYawEncoder();
    if(rotation > M_TWOPI) rotation = -(rotation - M_TWOPI);
    else if(rotation < -M_TWOPI) rotation = -(rotation + M_TWOPI);
    return rotation;
}


}//namespace cv