#include "cv_chassis.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"

#include "tap/communication/sensors/buzzer/buzzer.hpp"

namespace chassis
{
CVChassisCommand::CVChassisCommand(
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
void  CVChassisCommand::initialize() {
    chassis->setDesiredOutput(0, 0, 0);
}

void  CVChassisCommand::execute()
{
    if(drivers->cv_com.getChassisReadFlag()){
        //gets current cos and sin of yaw angle from starting point of gimbal
        float cosYaw = cosf(gimbalInterface->getYawEncoder());
        float sinYaw = sinf(gimbalInterface->getYawEncoder());
        //gets the cv inputs if valid
        float xInput = drivers->cv_com.getChassisX();
        float yInput = drivers->cv_com.getChassisY();
        //invalidate flag
        drivers->cv_com.resetChassisReadFlag();
        //applies rotation matrix to inputs to change inputs based on gimbal position
        float xOutput = ((cosYaw * xInput) - (sinYaw * yInput));
        float yOutput = ((cosYaw * yInput) + (sinYaw * xInput));
        //sends values to the chassis subsystem
        chassis->setDesiredOutput(
            xOutput,
            yOutput,
            drivers->cv_com.getChassisR());
    }
}

//stops movement again
void  CVChassisCommand::end(bool) {
    chassis->setDesiredOutput(0, 0, 0);
    }

bool  CVChassisCommand::isFinished() const { return false; }
}  // namespace chassis