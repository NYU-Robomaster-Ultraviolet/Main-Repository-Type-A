#include "cv_shooter.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"

namespace shooter
{
CVShooterCommand::CVShooterCommand(
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

void  CVShooterCommand::initialize() {}

void  CVShooterCommand::execute()
{
    if((drivers->cv_com.foundTarget() && !flyWheelsOn)){
        shooter->changeOnFlag();
        flyWheelsOn = !flyWheelsOn;
    }
    shooter->setDesiredOutput(0.3f);
}

void  CVShooterCommand::end(bool) {drivers->music_player.clearNote(); }

bool  CVShooterCommand::isFinished() const { return false; }
}  // namespace shooter