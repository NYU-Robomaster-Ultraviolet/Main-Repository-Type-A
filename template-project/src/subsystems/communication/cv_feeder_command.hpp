#ifndef CV_FEEDER_COMMAND_HPP_
#define CV_FEEDER_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "subsystems/feeder/feeder_subsystem.hpp"
#include "drivers.hpp"

#if defined (TARGET_STANDARD)
#include "controls/standard/standard_constants.hpp"
#elif defined (TARGET_SENTRY)
#include "controls/sentry/sentry_constants.hpp"
#elif defined (TARGET_HERO)
#include "controls/hero/hero_constants.hpp"
#endif

namespace feeder{

class CVFeeder : public tap::control::Command{
public:
    /**
     * Initializes the command with the passed in FeederSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] feeder a pointer to the feeder motorto be passed in that this
     *      Command will interact with.
     */
    CVFeeder(FeederSubsystem *const feeder, src::Drivers *drivers);

    CVFeeder(const CVFeeder &other) = delete;

    CVFeeder &operator=(const CVFeeder &other) = delete;

    void initialize() override;

    const char *getName() const { return "feeder CV command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;
private:
    FeederSubsystem *const feeder;

    src::Drivers *drivers;
}; //class CVFeeder : public tap::control::Command

} //namespace Feeder

#endif //FEEDER_MOVEMENT_COMMAND_HPP_ 