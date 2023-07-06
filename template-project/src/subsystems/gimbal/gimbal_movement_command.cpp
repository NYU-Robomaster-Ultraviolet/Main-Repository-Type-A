#include "gimbal_movement_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"

#include "tap/communication/sensors/imu/imu_interface.hpp"


namespace gimbal
{
GimbalMovementCommand::GimbalMovementCommand(GimbalSubsystem *const gimbal, src::Drivers *drivers, GimbalInterface *gimbalInt)
: gimbal(gimbal), drivers(drivers), gimbalInterface(gimbalInt)
{
     if (gimbal == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(gimbal));
}
void  GimbalMovementCommand::initialize() {
        gimbal->allignGimbal();
        if(!gimbal->isCalibrated()) timeout.restart(50);
        else timeout.restart(20);
    }

void  GimbalMovementCommand::execute()
{
    drivers->cv_com.setAngles(modm::toDegree(gimbal->getPitchEncoder() - PITCH_ENCODER_OFFSET), 
        modm::toDegree(gimbal->wrapAngle(gimbal->getYawEncoder() - YAW_ENCODER_OFFSET)));
    //sets beyblade mode
    gimbal->setBeybladeMode(gimbalInterface->getBeybladeMote());
    if(timeout.isExpired()){
        if(!gimbal->isCalibrated()) {
            drivers->mpu6500.requestCalibration();
            gimbal->calibrateImu();
            timeout.restart(50);
        }
        else if(drivers->mpu6500.getImuState() != tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATED){
            timeout.restart(50);
        }
        else{
            //drivers->leds.set(drivers->leds.A, drivers->mpu6500.getPitch() > 0);
            int yaw = int(gimbal->getImuYaw() * 100);
            int pitch = int(gimbal->getImuPitch() * 100);
            //drivers->cv_com.setAngles(pitch, yaw);
            gimbal->controllerInput(
            drivers->control_interface.getGimbalYawInput(),
            drivers->control_interface.getGimbalPitchInput());
        }
    }
    //drivers->music_player.playGivenNote(gimbal->getPitchEncoder() * 100);
}

void  GimbalMovementCommand::end(bool) {
    gimbal->controllerInput(0, 0);
    gimbal->noInputs();
    }

bool  GimbalMovementCommand::isFinished() const { return false; }


}//namespace gimbal