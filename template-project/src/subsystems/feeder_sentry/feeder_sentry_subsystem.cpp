#include "feeder_sentry_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#ifdef TARGET_SENTRY
//#include "tap/algorithms/math_user_utils.hpp"

using namespace tap;

namespace feeder
{

void FeederSentrySubsystem::initialize()
{
    feederMotor1.initialize();
    feederMotor2.initialize();
}

void FeederSentrySubsystem::refresh()
{
    updateFeederPid(&rpmPid1, &feederMotor1, motor1TargetRPM);
    updateFeederPid(&rpmPid2, &feederMotor2, motor2TargetRPM);
}

void FeederSentrySubsystem::updateFeederPid(
    modm::Pid<float>* pid,
    tap::motor::DjiMotor* const motor,
    float desiredRpm)
{
    if(desiredRpm || motor->getShaftRPM() > 50){
        pid->update(desiredRpm - motor->getShaftRPM());
        motor->setDesiredOutput(pid->getValue());
    }
    else motor->setDesiredOutput(0);
}

/*
    The target RPM should be a constant.
*/
void FeederSentrySubsystem::setTargetRPM(float RPM)
{
    motor1TargetRPM = RPM;
    motor2TargetRPM = RPM;
}
} //namespace feeder
#endif