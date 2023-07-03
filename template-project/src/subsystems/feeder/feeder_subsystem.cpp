#include "feeder_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
//#include "tap/algorithms/math_user_utils.hpp"

using namespace tap;

namespace feeder
{

void FeederSubsystem::initialize()
{
    #if defined (TARGET_SENTRY)
    feederMotor1.initialize();
    feederMotor2.initialize();
    #elif defined (TARGET_HERO) || (TARGET_STANDARD)
    feederMotor.initialize();
    #endif
}

void FeederSubsystem::refresh() {
    #if defined (TARGET_SENTRY)
    updateFeederPid(&rpmPid1, &feederMotor1, motor1TargetRPM);
    updateFeederPid(&rpmPid2, &feederMotor2, motor2TargetRPM);
    #elif defined (TARGET_HERO) || (TARGET_STANDARD)
    updateFeederPid(&rpmPid, &feederMotor, targetRPM);
    #endif
}

void FeederSubsystem::updateFeederPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRpm) {
    if(desiredRpm || motor->getShaftRPM() > 50){
        pid->update(desiredRpm - motor->getShaftRPM());
        motor->setDesiredOutput(pid->getValue());
    }
    else motor->setDesiredOutput(0);
    // drivers->leds.set(drivers->leds.E, feederMotor.isMotorOnline());
    // drivers->leds.set(drivers->leds.F, !feederMotor.isMotorOnline());
}   

/*
    The target RPM should be a constant.
*/
void FeederSubsystem::setTargetRPM(float RPM) 
{    
    #if defined (TARGET_SENTRY)
    motor1TargetRPM = RPM;
    motor2TargetRPM = RPM;
    #elif defined (TARGET_HERO) || (TARGET_STANDARD)
    targetRPM = RPM;
    #endif
}
} //namespace feeder