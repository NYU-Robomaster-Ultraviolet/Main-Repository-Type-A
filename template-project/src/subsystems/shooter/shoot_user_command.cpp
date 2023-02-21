#include "shoot_user_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/standard/control_interface.hpp"

namespace shooter
{
ShootUserCommand::ShootUserCommand(
    ShooterSubsystem *const shooter,
    src::Drivers *drivers)
    : shooter(shooter),
      drivers(drivers)
{
    if (shooter == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(shooter));
}

void  ShootUserCommand::initialize() {}

void  ShootUserCommand::execute()
{
    shooter->setDesiredOutput(0.3f);
}

void  ShootUserCommand::end(bool) {} //shooter->setDesiredOutput(0.2f); }

bool  ShootUserCommand::isFinished() const { return false; }
}  // namespace chassis