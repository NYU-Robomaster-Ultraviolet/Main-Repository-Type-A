#include "shooter_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"

using namespace tap;

namespace shooter
{

void ShooterSubsystem::initialize()
{
    drivers->pwm.write(0.25f, flywheel1);
    drivers->pwm.write(0.25f, flywheel2);
}
void ShooterSubsystem::refresh() {
    //setDesiredOutput(&flywheel1Ramp, flywheel1);
    //setDesiredOutput(&flywheel2Ramp, flywheel2);
}

float ShooterSubsystem::findRampOutput(float output)
{
    flywheelRamp.setTarget(output);
    flywheelRamp.update();
    return flywheelRamp.getValue();
}

void ShooterSubsystem::setDesiredOutput(float output) {
    float changeVal = findRampOutput(output);
    drivers->pwm.write(changeVal, flywheel2);
    drivers->pwm.write(changeVal, flywheel1);
}

/*
    Give desired setpoints for chassis movement. +x is forward, +y is right, +r is clockwise (turning right). Expressed in body frame.
*/
} //namespace chassis