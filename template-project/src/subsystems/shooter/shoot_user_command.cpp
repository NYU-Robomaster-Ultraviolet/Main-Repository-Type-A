#include "shoot_user_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"

namespace shooter
{
ShooterCommand::ShooterCommand(
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

void  ShooterCommand::initialize() {shooter->changeOnFlag();}

void  ShooterCommand::execute()
{
    //different measured speeds based on allowed bullet speeds at each level. Defaults level 1 if ref data isn't read
    uint8_t level = drivers->ref_interface.refDataValid() ? drivers->ref_interface.getLevel() : 1;
    // #if  defined (TARGET_STANDARD) || defined (TARGET_SENTRY)
    if(level == 3)
        shooter->setDesiredOutput(LEVEL_THREE_FLYWHEEL);
    else if(level == 2)
        shooter->setDesiredOutput(LEVEL_TWO_FLYWHEEL);
    else
        shooter->setDesiredOutput(LEVEL_ONE_FLYWHEEL);
    // #elif defined (TARGET_HERO)
    // shooter->setDesiredOutput(16000);
    // #endif
}

void  ShooterCommand::end(bool) {drivers->music_player.clearNote(); }

bool  ShooterCommand::isFinished() const { return false; }
}  // namespace shooter