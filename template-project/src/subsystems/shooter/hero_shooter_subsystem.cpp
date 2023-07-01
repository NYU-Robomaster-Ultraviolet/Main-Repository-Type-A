#ifdef TARGET_HERO
#include "hero_shooter_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"


using namespace tap;

namespace shooter
{

void HeroShooterSubsystem::initialize()
{
    flywheel1.initialize();
    flywheel2.initialize();
    flywheel1.setDesiredOutput(0);
    flywheel2.setDesiredOutput(0);
}
void HeroShooterSubsystem::refresh() {
    drivers->leds.set(drivers->leds.A, flywheel1Online());
    drivers->leds.set(drivers->leds.B, !flywheel1Online());
    drivers->leds.set(drivers->leds.C, flywheel2Online());
    drivers->leds.set(drivers->leds.D, !flywheel2Online());
    updateRpmPid(&pid1, &flywheel1, targetRPM);
    updateRpmPid(&pid2, &flywheel2, targetRPM);
}

float HeroShooterSubsystem::findRampOutput(float output)
{
    flywheelRamp.setTarget(output);
    flywheelRamp.update();
    return flywheelRamp.getValue();
}
void HeroShooterSubsystem::updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRpm) {
    pid->update(desiredRpm - motor->getShaftRPM()); //updates pid
    float output = pid->getValue();
    if(fabsf(output > 100))
        motor->setDesiredOutput(pid->getValue()); //feeds pid output value to motor
    else motor->setDesiredOutput(0);
}

void HeroShooterSubsystem::setDesiredOutput(float output) {
    if(stopFlag){
        targetRPM = output;
    }
    else targetRPM = 0;

}

} //namespace shooter
#endif