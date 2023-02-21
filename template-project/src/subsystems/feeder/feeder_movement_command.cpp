#include "feeder_movement_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/standard/control_interface.hpp"

namespace feeder
{
FeederMovementCommand::FeederMovementCommand(
    FeederSubsystem *const feeder,
    src::Drivers *drivers)
    : feeder(feeder),
      drivers(drivers)
{
    if (feeder == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(feeder));
}

void  FeederMovementCommand::initialize() {feeder->setTargetRPM(0);}

void  FeederMovementCommand::execute()
{
    feeder->setTargetRPM(1500);
    //if(feeder->motorOnline()) drivers->leds.set(drivers->leds.Blue, true);
    //else drivers->leds.set(drivers->leds.Red, true);
}

void  FeederMovementCommand::end(bool) { feeder->setTargetRPM(0); }

bool  FeederMovementCommand::isFinished() const { return false; }
}  // namespace feeder