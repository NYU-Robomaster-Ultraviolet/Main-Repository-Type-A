#ifndef CV_COMMAND_HPP_
#define CV_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "drivers.hpp"
#include "subsystems/gimbal/gimbal_subsystem.hpp"

#ifdef TARGET_STANDARD
#include "controls/standard/standard_constants.hpp"
#endif

#ifdef TARGET_SENTRY
#include "controls/sentry/sentry_constants.hpp"
#endif

#ifdef TARGET_HERO
#include "controls/hero/hero_constants.hpp"
#endif

namespace gimbal{

class CvCommand : public tap::control::Command {
public:
    CvCommand(GimbalSubsystem *const gimbal, src::Drivers *drivers);

    CvCommand(const CvCommand &other) = delete;

    CvCommand &operator=(const CvCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "cv command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    float findRotation(const float& destination) const;

private:
    GimbalSubsystem* gimbal;
    src::Drivers* drivers;
}; //CvCommand


}// namespace cv

#endif