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
    //drivers->cv_com.setEncoder(chassis->getRotationVelocity() * 100, chassis->getTransversal() * 100);

    //checks level and updates the level of chassis
    updateChassisLevel();
    //gets current cos and sin of yaw angle from starting point of gimbal
    float cosYaw = cosf(gimbalInterface->getYawEncoder());
    float sinYaw = sinf(gimbalInterface->getYawEncoder());
    //gets the controller inputs
    float xInput = limitVal<float>(drivers->control_interface.getChassisXInput(), -limitValueRange,
    limitValueRange);
    float yInput = limitVal<float>(drivers->control_interface.getChassisYInput(), -limitValueRange,
    limitValueRange);
    //applies rotation matrix to inputs to change inputs based on gimbal position
    float xOutput = ((cosYaw * xInput) - (sinYaw * yInput));
    float yOutput = ((cosYaw * yInput) + (sinYaw * xInput));

    if(!gimbalInterface->yawMotorOnline()){
        xOutput = xInput;
        yOutput = yInput;
    }
    
    //sends values to the chassis subsystem
    chassis->setDesiredOutput(
        xOutput,
        yOutput,
        limitVal<float>(drivers->control_interface.getChassisRotationInput(), -limitValueRange,
        limitValueRange));
        //0);
    //chassis->moveAllignWithGimbal();
}

//stops movement again
void  ChassisMovementCommand::end(bool) {
    chassis->setDesiredOutput(0, 0, 0);
    }

bool ChassisMovementCommand::checkPowerLimit(){
    if(drivers->ref_interface.refDataValid() && checkPowerTimeout.isExpired()){
        checkPowerTimeout.restart(500);
        std::pair<uint16_t, uint16_t> powerLimits = drivers->ref_interface.getPowerUsage();
        if(powerLimits.first > powerLimits.second * .9){
            limitValueRange -= .05;
        }
        else if(powerLimits.first < powerLimits.second * .8 && limitValueRange < 1){
            limitValueRange += .05;
        }
        return true;
    }
    return false;
}

bool  ChassisMovementCommand::isFinished() const { return false; }

bool ChassisMovementCommand::updateChassisLevel() {
    if(drivers->ref_interface.refDataValid()){
        chassis->setRobotLevel(drivers->ref_interface.getLevel());
        return true;
    }
    return false;
}
}  // namespace chassis