#include "chassis_beyblade.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"

#include "tap/communication/sensors/buzzer/buzzer.hpp"

namespace chassis
{
ChassisBeybladeCommand::ChassisBeybladeCommand(
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
void  ChassisBeybladeCommand::initialize() {
    chassis->setDesiredOutput(0, 0, 0);
    //chassis->setBeybladeMode(1);
}

void  ChassisBeybladeCommand::execute()
{

    //print second then first
    //drivers->cv_com.setEncoder(chassis->getRotationVelocity(), gimbalInterface->getYawVelocity());

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
        0);//-rotation);
}

//stops movement again
void  ChassisBeybladeCommand::end(bool) {
    chassis->setDesiredOutput(0, 0, 0);
    chassis->setBeybladeMode(0);
    }

bool  ChassisBeybladeCommand::isFinished() const { return false; }
}  // namespace chassis