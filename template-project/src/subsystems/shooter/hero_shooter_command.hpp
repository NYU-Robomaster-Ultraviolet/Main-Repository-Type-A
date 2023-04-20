#ifdef TARGET_HERO
#ifndef HERO_SHOOT_COMMAND_HPP_
#define HERO_SHOOT_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "hero_shooter_subsystem.hpp"
#include "drivers.hpp"
namespace shooter{

class HeroShooterCommand : public tap::control::Command{
public:
    /**
     * Initializes the command with the passed in ShooterSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] shooter a pointer to the shooter to be passed in that this
     *      Command will interact with.
     */
    HeroShooterCommand(HeroShooterSubsystem *const shooter, src::Drivers *drivers);

    HeroShooterCommand(const HeroShooterCommand &other) = delete;

    HeroShooterCommand &operator=(const HeroShooterCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "hero shooter command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    bool flag = false;
private:
    HeroShooterSubsystem *const shooter;

    src::Drivers *drivers;
}; //class ShooterUserCommand : public tap::control::Command
} //namespace shooter
#endif
#endif //SHOOTER_USER_COMMAND_HPP_ 