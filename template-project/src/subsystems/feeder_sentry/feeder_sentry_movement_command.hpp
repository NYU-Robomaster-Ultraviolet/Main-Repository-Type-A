#ifndef FEEDER_SENTRY_MOVEMENT_COMMAND_HPP_
#define FEEDER_SENTRY_MOVEMENT_COMMAND_HPP_
#ifdef TARGET_SENTRY

#include "tap/control/command.hpp"

#include "feeder_sentry_subsystem.hpp"
#include "drivers.hpp"
#include "tap/architecture/timeout.hpp"
namespace feeder{

class FeederSentryMovementCommand : public tap::control::Command{
public:
    /**
     * Initializes the command with the passed in FeederSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] feeder a pointer to the feeder motorto be passed in that this
     *      Command will interact with.
     */
    FeederSentryMovementCommand(FeederSentrySubsystem *const feeder, src::Drivers *drivers);

    FeederSentryMovementCommand(const FeederSentryMovementCommand &other) = delete;

    FeederSentryMovementCommand &operator=(const FeederSentryMovementCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "feeder drive command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;
private:
    FeederSentrySubsystem *const feeder;
    tap::arch::MilliTimeout burstFireTimeout;
    src::Drivers *drivers;
}; //class FeederSentryMovementCommand : public tap::control::Command

} //namespace Feeder

#endif //FEEDER_SENTRY_MOVEMENT_COMMAND_HPP_
#endif