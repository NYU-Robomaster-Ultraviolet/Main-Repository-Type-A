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
    checkPowerTimeout.restart(10);
    #if defined (TARGET_SENTRY)
    switchTurnTimeout.restart(10);
    #endif
}

void  ChassisBeybladeCommand::execute()
{   
    #if defined (TARGET_SENTRY)
    if(drivers->ref_interface.gameStarted() && drivers->ref_interface.spentMoney() && !spent_already)
    {
        spent_already = true;
    }
    if(!drivers->ref_interface.gameStarted() && !spent_already)
    {
        return;
    }
    #endif
    //if(drivers->ref_interface.)
    //checks level and updates the level of chassis
    updateChassisLevel();
    //print second then first
    //drivers->cv_com.setEncoder(chassis->getRotationVelocity(), gimbalInterface->getYawVelocity());

    //gets current cos and sin of yaw angle from starting point of gimbal
    float cosYaw = cosf(gimbalInterface->getYawEncoder());
    float sinYaw = sinf(gimbalInterface->getYawEncoder());

    //check on power consumption
    checkPowerLimits();

    //update beyblade mode
    gimbalInterface->setBeyblade(true);
    
    //gets the controller inputs
    float xInput = drivers->control_interface.getChassisXInput();
    float yInput = drivers->control_interface.getChassisYInput();
    //applies rotation matrix to inputs to change inputs based on gimbal position
    float xOutput = ((cosYaw * xInput) - (sinYaw * yInput));
    float yOutput = ((cosYaw * yInput) + (sinYaw * xInput));
    #if defined (TARGET_SENTRY)
    if(switchTurnTimeout.isExpired()) {
        turningRight = !turningRight;
        switchTurnTimeout.restart(timeoutInterval);
    }
    // if(turningRight) yOutput = inputVal;
    // else yOutput = -inputVal;
    if(turningRight) xOutput = inputVal;
    else xOutput = -inputVal;
    #endif 
    //sends values to the chassis subsystem
    chassis->setDesiredOutput(
        xOutput * .8, //limits movement when beyblading
        yOutput * .8,
        rotation * gimbalInterface->getSlowBeyblade());
}

//stops movement again
void  ChassisBeybladeCommand::end(bool) {
    chassis->setDesiredOutput(0, 0, 0);
    chassis->setBeybladeMode(0);
     gimbalInterface->setBeyblade(false);
    }

bool  ChassisBeybladeCommand::isFinished() const { 
    if(drivers->ref_interface.gameFinished())
    {
        return true;
    }
    return false; 
}

bool ChassisBeybladeCommand::checkPowerLimits(){
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

bool ChassisBeybladeCommand::updateChassisLevel() {
    if(drivers->ref_interface.refDataValid()){
        robotLevel = drivers->ref_interface.getLevel();
        if(robotLevel == 2) rotation = BEYBLADE_INPUT_TWO;
        else if(robotLevel == 3) rotation = BEYBLADE_INPUT_THREE;
        else rotation = BEYBLADE_INPUT;
        
        chassis->setRobotLevel(robotLevel);
        return true;
    }
    return false;
}
}  // namespace chassis