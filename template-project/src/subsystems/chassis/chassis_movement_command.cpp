#include "chassis_movement_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"

#include "tap/communication/sensors/buzzer/buzzer.hpp"

namespace chassis
{
ChassisMovementCommand::ChassisMovementCommand(
    ChassisSubsystem *const chassis,
    src::Drivers *drivers, gimbal::GimbalInterface* gimbal)
    : chassis(chassis),
      drivers(drivers),
      gimbalInterface(gimbal)
{
    if (chassis == nullptr)  return; // checks if subsystem exists
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(chassis)); //set requirement
}

//stop any movement
void  ChassisMovementCommand::initialize() {
    chassis->setDesiredOutput(0, 0, 0);
}

void  ChassisMovementCommand::execute()
{
    //print second then first
    drivers->cv_com.setEncoder(chassis->getRotationVelocity() * 100, chassis->getTransversal() * 100);

    //gets current cos and sin of yaw angle from starting point of gimbal
    float cosYaw = cosf(gimbalInterface->getYawEncoder());
    float sinYaw = sinf(gimbalInterface->getYawEncoder());
    //gets the controller inputs
    float xInput = drivers->control_interface.getChassisXInput();
    float yInput = drivers->control_interface.getChassisYInput();
    //applies rotation matrix to inputs to change inputs based on gimbal position
    float xOutput = ((cosYaw * xInput) - (sinYaw * yInput));
    float yOutput = ((cosYaw * yInput) + (sinYaw * xInput));
    //sends values to the chassis subsystem
    chassis->setDesiredOutput(
        xOutput,
        yOutput,
        drivers->control_interface.getChassisRotationInput());
        //0);
    //chassis->moveAllignWithGimbal();
}

//stops movement again
void  ChassisMovementCommand::end(bool) {
    chassis->setDesiredOutput(0, 0, 0);
    }

void ChassisMovementCommand::checkPowerLimit(){
    std::pair<uint16_t, uint16_t> limits = drivers->ref_interface.getPowerUsage();
    if(limits.first / limits.second > .9){
        chassis->limitPower(.9);
    }
    //else if(limits.first / limits.second < .2 )
}

bool  ChassisMovementCommand::isFinished() const { return false; }
}  // namespace chassis