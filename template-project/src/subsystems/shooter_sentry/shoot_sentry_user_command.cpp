#include "shoot_sentry_user_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"

namespace shooter
{
ShootSentryUserCommand::ShootSentryUserCommand(
    ShooterSentrySubsystem *const shooter,
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

void  ShootSentryUserCommand::initialize() {
    shooter->setDesiredOutput(.25f);
    shooter->changeOnFlag();
}

void  ShootSentryUserCommand::execute()
{shooter->setDesiredOutput(0.3f);
    drivers->music_player.execute();
    if(drivers->music_player.finishedSong()){
        drivers->music_player.resetSong();
    }
}

void  ShootSentryUserCommand::end(bool) {drivers->music_player.clearNote();}//shooter->setDesiredOutput(0.2f); }

bool  ShootSentryUserCommand::isFinished() const { return false; }
}  // namespace shooter