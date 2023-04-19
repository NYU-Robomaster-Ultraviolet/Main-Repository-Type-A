#include "gimbal_beyblade.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"


namespace gimbal
{
GimbalBeybladeCommand::GimbalBeybladeCommand(GimbalSubsystem *const gimbal, src::Drivers *drivers)
: gimbal(gimbal), drivers(drivers)
{
     if (gimbal == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(gimbal));
}
void  GimbalBeybladeCommand::initialize() {
        gimbal->cvInput(findRotation(YAW_ENCODER_OFFSET), LEVEL_ANGLE - gimbal->getPitchEncoder());
        noTurn = 0;
    }

void  GimbalBeybladeCommand::execute()
{
    gimbal->setIMU(drivers->imu_rad_interface.getYaw(), drivers->imu_rad_interface.getPitch());
    gimbal->controllerInput(drivers->control_interface.getGimbalYawInput() - rotation,
        drivers->control_interface.getGimbalPitchInput());
}

void  GimbalBeybladeCommand::end(bool) {
    gimbal->controllerInput(0, 0);
    gimbal->noInputs();
    }

bool  GimbalBeybladeCommand::isFinished() const { return false; }

float GimbalBeybladeCommand::findRotation(const float& destination) const {
    float rotation = destination - gimbal->getYawEncoder();
    if(rotation > M_TWOPI) rotation = -(rotation - M_TWOPI);
    else if(rotation < -M_TWOPI) rotation = -(rotation + M_TWOPI);
    return rotation;
}

}//namespace gimbal