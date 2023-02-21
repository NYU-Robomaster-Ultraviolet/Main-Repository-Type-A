#include "feeder_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
//#include "tap/algorithms/math_user_utils.hpp"

using namespace tap;

namespace feeder
{

void FeederSubsystem::initialize()
{
    feederMotor1.initialize();
    feederMotor2.initialize();
}

void FeederSubsystem::refresh() {
    updateFeederPid(&rpmPid, &feederMotor1, &feederMotor2, targetRPM);
}

void FeederSubsystem::updateFeederPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor1, tap::motor::DjiMotor* const motor2, float desiredRpm) {
    if(desiredRpm || motor1->getShaftRPM() > 50 || motor2->getShaftRPM() > 50){
        pid->update(desiredRpm - motor1->getShaftRPM());
        pid->update(desiredRpm - motor2->getShaftRPM());
        motor1->setDesiredOutput(pid->getValue());
        motor2->setDesiredOutput(pid->getValue()); // 
    }
    else {
        motor1->setDesiredOutput(0);
        motor2->setDesiredOutput(0);
    }
}   

/*
    The target RPM should be a constant.
*/
void FeederSubsystem::setTargetRPM(float RPM) 
{    
    targetRPM = RPM;
}
} //namespace feeder