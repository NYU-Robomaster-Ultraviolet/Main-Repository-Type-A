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
    //check if stop command
    if(drivers->cv_com.getChassisStop()){
        chassis->changeVelocityMoveFlag(false);
        chassis->setDesiredOutput(0, 0, 0);
    }
    else if(drivers->cv_com.getChassisReadFlag()){
        //gets current cos and sin of yaw angle from starting point of gimbal
        float cosYaw = cosf(gimbalInterface->getYawEncoder());
        float sinYaw = sinf(gimbalInterface->getYawEncoder());
        //gets the cv inputs if valid
        float xInput = limitVal<float>(drivers->cv_com.getChassisX() + drivers->control_interface.getChassisXInput(), -1, 1);
        float yInput = limitVal<float>(drivers->cv_com.getChassisY() + drivers->control_interface.getChassisYInput(), -1, 1);
        float rInput = limitVal<float>(drivers->cv_com.getChassisR() + drivers->control_interface.getChassisRotationInput(), -1, 1);
        //invalidate flag
        drivers->cv_com.resetChassisReadFlag();
        //applies rotation matrix to inputs to change inputs based on gimbal position
        float xOutput = ((cosYaw * xInput) - (sinYaw * yInput));
        float yOutput = ((cosYaw * yInput) + (sinYaw * xInput));
        //sends values to the chassis subsystem
        chassis->setDesiredOutput( xOutput, yOutput,rInput);
        chassis->changeVelocityMoveFlag(false);
    }
    else{
        if(drivers->cv_com.getChassisVeloFlag()){
            chassis->setTargetVelocity(drivers->cv_com.getChassisFowardVelo(), drivers->cv_com.getChassisRightVelo());
            drivers->cv_com.resetChassisVeloFlag();
            chassis->changeVelocityMoveFlag(true);
        }
        if(drivers->cv_com.getChassisSpinFlag()){
            chassis->setRotationVelocity(drivers->cv_com.getChassisRotationVelo());
            drivers->cv_com.resetChassisSpinFlag();
            chassis->changeVelocityMoveFlag(true);
        }
        if(drivers->cv_com.getChassisFowardFlag()){
            chassis->setFowardMovement(drivers->cv_com.getChassisFowardMovement());
            drivers->cv_com.resetChassisFowardFlag();
            chassis->changeVelocityMoveFlag(false);
        }
        if(drivers->cv_com.getChassisSpinRadFlag()){
            chassis->setRotationRadians(drivers->cv_com.getChassisSpinRad());
            drivers->cv_com.resetChassisSpinRadFlag();
            chassis->changeVelocityMoveFlag(false);
        }
    }
}

//stops movement again
void  CVChassisCommand::end(bool) {
    chassis->setDesiredOutput(0, 0, 0);
    chassis->changeVelocityMoveFlag(false);
    }

bool  CVChassisCommand::isFinished() const { return false; }
}  // namespace chassis