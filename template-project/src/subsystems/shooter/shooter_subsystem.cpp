#include "shooter_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"

using namespace tap;

namespace shooter
{

void ShooterSubsystem::initialize()
{
   initializeFlywheel();
}
void ShooterSubsystem::refresh() {
    #if defined (TARGET_HERO)
        updateRpmPid(&pid1, &flywheel1, targetRPM);
        updateRpmPid(&pid2, &flywheel2, targetRPM);
    #endif
}

#if defined (TARGET_SENTRY) || defined (TARGET_STANDARD)
float ShooterSubsystem::findRampOutput(float output)
{
    flywheelRamp.setTarget(output);
    flywheelRamp.update();
    return flywheelRamp.getValue();
}
#endif

#if defined (TARGET_HERO)
void ShooterSubsystem::updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRpm) {
    float error = desiredRpm - motor->getShaftRPM();
    if(error < 5000) motor->setDesiredOutput(0);
    else{
        pid->update(error); //updates pid
        float output = pid->getValue();
        if(fabsf(output > 1000))
            motor->setDesiredOutput(pid->getValue()); //feeds pid output value to motor
        else motor->setDesiredOutput(0);
    }
}
#endif

void ShooterSubsystem::setDesiredOutput(float output) {
    #if defined (TARGET_SENTRY) || defined (TARGET_STANDARD)
    if(!on) output = .2;
    float changeVal = findRampOutput(output);
    drivers->pwm.write(changeVal, flywheel2);
    drivers->pwm.write(changeVal, flywheel1);
    #endif
    #if defined (TARGET_SENTRY)
    drivers->pwm.write(changeVal, flywheel4);
    drivers->pwm.write(changeVal, flywheel3);
    #elif defined (TARGET_HERO)
    if(on){
        targetRPM = output;
        }
    else targetRPM = 0;
    #endif
    }

    void ShooterSubsystem::initializeFlywheel(){
        online = true;
         #if defined (TARGET_SENTRY) || defined (TARGET_STANDARD)
        drivers->pwm.write(0.25f, flywheel1);
        drivers->pwm.write(0.25f, flywheel2);
        #endif
        //add 2 more motors for sentry, or use dji motors instead if hero
        #if defined (TARGET_SENTRY)
        drivers->pwm.write(0.25f, flywheel3);
        drivers->pwm.write(0.25f, flywheel4);
        #elif defined (TARGET_HERO)
        if(!initialized) {
            flywheel1.initialize();
            flywheel2.initialize();
            initialized = true;
        }
        flywheel1.setDesiredOutput(0);
        flywheel2.setDesiredOutput(0);
        #endif
    }
} //namespace shooter