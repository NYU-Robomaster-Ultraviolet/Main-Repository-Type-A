#include "first_move.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"

#include "tap/communication/sensors/buzzer/buzzer.hpp"

namespace chassis
{
FirstMove::FirstMove(
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
void  FirstMove::initialize() {
    chassis->setDesiredOutput(0, 0, 0);
    timeout.restart(10);
    // chassis->setRotationRadians(drivers->cv_com.getChassisSpinRad());
    // chassis->setRotationVelocity(drivers->cv_com.getChassisRotationVelo());
    // chassis->changeVelocityMoveFlag(false);
}

void  FirstMove::execute()
{
    if(!drivers->ref_interface.gameStarted()) return;
    if(chassis->getTargetDistance() || chassis->getTargetRotation()) return;
    if(!movingFoward){
        chassis->setTargetVelocity(3, 0);
        chassis->setFowardMovement(10000);
        movingFoward = true;
    }
    else if(movingFoward && !rotating && !finishedRotation){
        chassis->setRotationVelocity(2);
        chassis->setRotationRadians(M_PI_2);
        rotating = true;
    }
    else if(!chassis->getTargetRotation() && rotating){
        finishedMovements = true;
    }
}

//stops movement again
void  FirstMove::end(bool) {
    chassis->setFowardMovement(0);
    chassis->setRotationRadians(0);
    chassis->setRotationVelocity(0);
    chassis->setTargetVelocity(0, 0);
    chassis->setDesiredOutput(0, 0, 0);
    chassis->changeVelocityMoveFlag(false);
    }

bool  FirstMove::isFinished() const { return finishedMovements; }

bool FirstMove::checkPowerLimit(){
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
}  // namespace chassis