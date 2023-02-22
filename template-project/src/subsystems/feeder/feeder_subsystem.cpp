#include "feeder_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
//#include "tap/algorithms/math_user_utils.hpp"

using namespace tap;

namespace feeder
{

void FeederSubsystem::initialize()
{
    feederMotor.initialize();
}

void FeederSubsystem::refresh() {
    updateFeederPid(&rpmPid, &feederMotor, targetRPM);
}

void FeederSubsystem::updateFeederPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRpm) {
    if(desiredRpm || motor->getShaftRPM() > 50){
        pid->update(desiredRpm - motor->getShaftRPM());
        motor->setDesiredOutput(pid->getValue());
    }
    else motor->setDesiredOutput(0);
}   

/*
    The target RPM should be a constant.
*/
void FeederSubsystem::setTargetRPM(float RPM) 
{    
    targetRPM = RPM;
}
} //namespace feeder