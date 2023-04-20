#include "feeder_sentry_movement_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"
#ifdef TARGET_SENTRY

namespace feeder
{
FeederSentryMovementCommand::FeederSentryMovementCommand(
    FeederSentrySubsystem *const feeder,
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

void  FeederSentryMovementCommand::initialize() {feeder->setTargetRPM(0);}

void  FeederSentryMovementCommand::execute()
{
    feeder->setTargetRPM(1500);
}

void  FeederSentryMovementCommand::end(bool) { feeder->setTargetRPM(0); }

bool  FeederSentryMovementCommand::isFinished() const { return false; }
}  // namespace feeder
#endif