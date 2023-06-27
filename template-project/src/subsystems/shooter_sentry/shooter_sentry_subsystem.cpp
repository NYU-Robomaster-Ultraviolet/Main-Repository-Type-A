#include "shooter_sentry_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"

using namespace tap;

namespace shooter
{

void ShooterSentrySubsystem::initialize()
{
    drivers->pwm.write(0.25f, flywheel1);
    drivers->pwm.write(0.25f, flywheel2);
    drivers->pwm.write(0.25f, flywheel3);
    drivers->pwm.write(0.25f, flywheel4);
}
void ShooterSentrySubsystem::refresh() {
    //setDesiredOutput(&flywheel1Ramp, flywheel1);
    //setDesiredOutput(&flywheel2Ramp, flywheel2);
}

float ShooterSentrySubsystem::findRampOutput(float output)
{
    flywheelRamp.setTarget(output);
    flywheelRamp.update();
    return flywheelRamp.getValue();
}

void ShooterSentrySubsystem::setDesiredOutput(float output) {
    if(!on) output = .2;
    float changeVal = findRampOutput(output);
    drivers->pwm.write(changeVal, flywheel4);
    drivers->pwm.write(changeVal, flywheel3);
    drivers->pwm.write(changeVal, flywheel2);
    drivers->pwm.write(changeVal, flywheel1);
}

} //namespace shooter