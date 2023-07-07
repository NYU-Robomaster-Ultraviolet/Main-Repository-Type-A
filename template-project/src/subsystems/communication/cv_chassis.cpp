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
    timeout.restart(10);
    // chassis->setRotationRadians(drivers->cv_com.getChassisSpinRad());
    // chassis->setRotationVelocity(drivers->cv_com.getChassisRotationVelo());
    // chassis->changeVelocityMoveFlag(false);
}

void  CVChassisCommand::execute()
{
    
    if(!drivers->ref_interface.gameStarted()) return;
    //checks level and updates the level of chassis
    updateChassisLevel();
    
    //get remote inputs
    float xInput = limitVal<float>(drivers->control_interface.getChassisXInput(), -1, 1);
    float yInput = limitVal<float>(drivers->control_interface.getChassisYInput(), -1, 1);
    float rInput = limitVal<float>(drivers->control_interface.getChassisRotationInput(), -1, 1);
    
    float xOutput = 0;
    float yOutput = 0;
    float rOutput = 0;
    unsigned char beyblade = drivers->cv_com.getBeybladeMode();

    //gets current cos and sin of yaw angle from starting point of gimbal
    float cosYaw = cosf(gimbalInterface->getYawEncoder());
    float sinYaw = sinf(gimbalInterface->getYawEncoder());
    if(beyblade == 1)
        rOutput += beybladeInput;
    else if(beyblade == 2)
        rOutput -= beybladeInput;

    //print first then second
    //drivers->cv_com.setEncoder(chassis->getTargetRotation() * 100, chassis->getRotationVelocity() * 100);
    
    //check if there are inputs
    if(xInput || yInput || (rInput && beyblade == 0)){
        xOutput = ((cosYaw * xInput) - (sinYaw * yInput));
        yOutput = ((cosYaw * yInput) + (sinYaw * xInput));
        rOutput += rInput;
        // chassis->changeVelocityMoveFlag(false);
        // chassis->setFowardMovement(0);
        // chassis->setRotationRadians(0);
        // chassis->setRotationVelocity(0);
        // chassis->setTargetVelocity(0, 0);
        chassis->setDesiredOutput( xOutput, yOutput, rOutput);
    }
    //check if stop command
    else if(drivers->cv_com.getChassisStop()){
        // chassis->changeVelocityMoveFlag(false);
        xOutput = 0;
        yOutput = 0;
        rOutput = 0;
        // chassis->setFowardMovement(0);
        // chassis->setRotationRadians(0);
        // chassis->setRotationVelocity(0);
        // chassis->setTargetVelocity(0, 0);
        chassis->setDesiredOutput( xOutput, yOutput, rOutput);
    }
    else if(drivers->cv_com.getChassisReadFlag()){

        //gets the cv inputs if valid
        xInput = limitVal<float>(drivers->cv_com.getChassisX(), -limitValueRange, limitValueRange);
        yInput = limitVal<float>(drivers->cv_com.getChassisY(), -limitValueRange, limitValueRange);
        rInput = limitVal<float>(drivers->cv_com.getChassisR(), -limitValueRange, limitValueRange);
        //invalidate flag
        drivers->cv_com.resetChassisReadFlag();
        //applies rotation matrix to inputs to change inputs based on gimbal position
        xOutput = ((cosYaw * xInput) - (sinYaw * yInput));
        yOutput = ((cosYaw * yInput) + (sinYaw * xInput));
        // if(rInput)
        //     rOutput = rInput;
        chassis->setDesiredOutput( xOutput, yOutput, rOutput);
        //sends values to the chassis subsystem
        //chassis->changeVelocityMoveFlag(false);
        //chassis->setDesiredOutput( xOutput, yOutput, rOutput);
    }
    // else if(drivers->cv_com.getChassisPowerFlag()){
    //     //gets the cv inputs if valid
    //     xInput = limitVal<float>(drivers->cv_com.getYPower(), -1, 1);
    //     yInput = limitVal<float>(drivers->cv_com.getXPower(), -1, 1);
    //     //invalidate flag
    //     drivers->cv_com.resetChassisPowerFlag();
    //     //applies rotation matrix to inputs to change inputs based on gimbal position
    //     float xOutput = ((cosYaw * xInput) - (sinYaw * yInput));
    //     float yOutput = ((cosYaw * yInput) + (sinYaw * xInput));
    //     //sends values to the chassis subsystem
    //     chassis->setDesiredOutput( xOutput, yOutput, 0);
    //     chassis->changeVelocityMoveFlag(false);
    // }
    // else{
    //     if(drivers->cv_com.getChassisVeloFlag()){
    //         chassis->setTargetVelocity(drivers->cv_com.getChassisFowardVelo(), drivers->cv_com.getChassisRightVelo());
    //         drivers->cv_com.resetChassisVeloFlag();
    //         chassis->changeVelocityMoveFlag(true);
    //     }
    //     if(drivers->cv_com.getChassisSpinFlag()){
    //         chassis->setRotationVelocity(drivers->cv_com.getChassisRotationVelo());
    //         drivers->cv_com.resetChassisSpinFlag();
    //         chassis->changeVelocityMoveFlag(true);
    //     }
    //     if(drivers->cv_com.getChassisFowardFlag()){
    //         chassis->setFowardMovement(drivers->cv_com.getChassisFowardMovement());
    //         chassis->setTargetVelocity(drivers->cv_com.getChassisFowardVelo(), 0);
    //         drivers->cv_com.resetChassisFowardFlag();
    //         chassis->changeVelocityMoveFlag(false);
    //     }
    //     if(drivers->cv_com.getChassisSpinRadFlag()){
    //         // chassis->setRotationRadians(drivers->cv_com.getChassisSpinRad());
    //         // drivers->cv_com.resetChassisSpinRadFlag();
    //         // chassis->changeVelocityMoveFlag(false);
    //     }
    // }
}

//stops movement again
void  CVChassisCommand::end(bool) {
    chassis->setFowardMovement(0);
    chassis->setRotationRadians(0);
    chassis->setRotationVelocity(0);
    chassis->setTargetVelocity(0, 0);
    chassis->setDesiredOutput(0, 0, 0);
    chassis->changeVelocityMoveFlag(false);
    }

bool  CVChassisCommand::isFinished() const { return false; }

bool CVChassisCommand::checkPowerLimit(){
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

bool CVChassisCommand::updateChassisLevel() {
    
    if(drivers->ref_interface.refDataValid()){
        uint8_t level = drivers->ref_interface.getLevel();
        if(level == 3) beybladeInput = BEYBLADE_INPUT_THREE;
        else if(level == 2) beybladeInput = BEYBLADE_INPUT_TWO;
        else beybladeInput = BEYBLADE_INPUT;
        chassis->setRobotLevel(level);
        return true;
    }
    return false;
}

}  // namespace chassis