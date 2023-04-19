#include "gimbal_movement_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"


namespace gimbal
{
GimbalMovementCommand::GimbalMovementCommand(GimbalSubsystem *const gimbal, src::Drivers *drivers)
: gimbal(gimbal), drivers(drivers)
{
     if (gimbal == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(gimbal));
}
void  GimbalMovementCommand::initialize() {
        gimbal->cvInput(findRotation(YAW_ENCODER_OFFSET), LEVEL_ANGLE - gimbal->getPitchEncoder());
        if(!gimbal->isCalibrated()) timeout.restart(2500);
        else timeout.restart(20);
    }

void  GimbalMovementCommand::execute()
{
    if(timeout.isExpired()){
        if(!gimbal->isCalibrated()) {
            drivers->mpu6500.requestCalibration();
            gimbal->calibrateImu();
            timeout.restart(50);
        }
        else{
            //drivers->leds.set(drivers->leds.A, drivers->mpu6500.getPitch() > 0);
            gimbal->setIMU(drivers->imu_rad_interface.getYaw(), drivers->imu_rad_interface.getPitch());
            gimbal->controllerInput(drivers->control_interface.getGimbalYawInput(),
                drivers->control_interface.getGimbalPitchInput());
        }
    }
}

void  GimbalMovementCommand::end(bool) {
    gimbal->controllerInput(0, 0);
    gimbal->noInputs();
    }

bool  GimbalMovementCommand::isFinished() const { return false; }

float GimbalMovementCommand::findRotation(const float& destination) const {
    float rotation = destination - gimbal->getYawEncoder();
    if(rotation > M_TWOPI) rotation = -(rotation - M_TWOPI);
    else if(rotation < -M_TWOPI) rotation = -(rotation + M_TWOPI);
    return rotation;
}

}//namespace gimbal