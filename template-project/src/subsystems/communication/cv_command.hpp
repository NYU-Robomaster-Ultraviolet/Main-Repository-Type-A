#ifndef CV_COMMAND_HPP_
#define CV_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "drivers.hpp"
#include "subsystems/gimbal/gimbal_subsystem.hpp"

#if defined (TARGET_STANDARD)
#include "controls/standard/standard_constants.hpp"
#elif defined (TARGET_SENTRY)
#include "controls/sentry/sentry_constants.hpp"
#elif defined (TARGET_HERO)
#include "controls/hero/hero_constants.hpp"
#endif

namespace gimbal{

class CVGimbal : public tap::control::Command {
public:
    CVGimbal(GimbalSubsystem *const gimbal, src::Drivers *drivers);

    CVGimbal(const CVGimbal &other) = delete;

    CVGimbal &operator=(const CVGimbal &other) = delete;

    void initialize() override;

    const char *getName() const { return "cv command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;


private:
    GimbalSubsystem* gimbal;
    src::Drivers* drivers;

}; //CVGimbal


}// namespace cv

#endif