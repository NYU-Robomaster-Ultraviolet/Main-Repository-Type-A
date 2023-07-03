#include "shoot_user_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"

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

void  ShootUserCommand::initialize() {shooter->changeOnFlag();}

void  ShootUserCommand::execute()
{
    drivers->music_player.execute();
    if(drivers->music_player.finishedSong()){
        drivers->music_player.resetSong();
    }
    #if  defined (TARGET_STANDARD) || defined (TARGET_SENTRY)
    shooter->setDesiredOutput(0.3f);
    #elif defined (TARGET_HERO)
    shooter->setDesiredOutput(16000);
    #endif
}

void  ShootUserCommand::end(bool) {drivers->music_player.clearNote(); }

bool  ShootUserCommand::isFinished() const { return false; }
}  // namespace shooter