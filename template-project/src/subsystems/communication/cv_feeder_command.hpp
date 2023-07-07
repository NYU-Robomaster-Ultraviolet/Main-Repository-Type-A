#ifndef CV_FEEDER_COMMAND_HPP_
#define CV_FEEDER_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "subsystems/feeder/feeder_subsystem.hpp"
#include "subsystems/shooter/shooter_Interface.hpp"
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

    /**
     * @brief stops any feeder movement and resets timers
     * 
     * @return ** void 
     */
    void initialize() override;

    const char *getName() const { return "feeder CV command"; }

    /**
     * @brief Waits for cv to indicate that a target has been found, if so, fire for minimum of 1 second before checking
     * if the target is still valid. If approaching barrel heat limit or target lost, stop firing.
     * 
     * @return ** void 
     */
    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    //returns if you are about to hit the heat limit or not, true for reaching limit, false if not
    bool checkBarrelHeatLimit() const;
private:
    FeederSubsystem *const feeder;

    tap::arch::MilliTimeout burstFireTimeout;
    tap::arch::MilliTimeout burstFireCooldown;

    src::Drivers *drivers;
}; //class CVFeeder : public tap::control::Command

} //namespace Feeder

#endif //FEEDER_MOVEMENT_COMMAND_HPP_ 