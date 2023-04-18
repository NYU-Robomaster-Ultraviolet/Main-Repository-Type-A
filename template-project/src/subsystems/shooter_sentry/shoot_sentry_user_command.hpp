#ifndef SHOOT_SENTRY_MOVEMENT_COMMAND_HPP_
#define SHOOT_SENTRY_MOVEMENT_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "shooter_sentry_subsystem.hpp"
#include "drivers.hpp"
namespace shooter{

class ShootSentryUserCommand : public tap::control::Command{
public:
    /**
     * Initializes the command with the passed in ShooterSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] shooter a pointer to the shooter to be passed in that this
     *      Command will interact with.
     */
    ShootSentryUserCommand(ShooterSentrySubsystem *const shooter, src::Drivers *drivers);

    ShootSentryUserCommand(const ShootSentryUserCommand &other) = delete;

    ShootSentryUserCommand &operator=(const ShootSentryUserCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "chassis drive command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;
private:
    ShooterSentrySubsystem *const shooter;

    src::Drivers *drivers;
}; //class ShooterSentryUserCommand : public tap::control::Command
} //namespace shooter
#endif  // SHOOT_SENTRY_MOVEMENT_COMMAND_HPP_