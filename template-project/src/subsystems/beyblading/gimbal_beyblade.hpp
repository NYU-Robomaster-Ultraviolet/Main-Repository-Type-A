#ifndef GIMBAL_BEYBLADE_HPP_
#define GIMBAL_BEYBLADE_HPP_

#include "tap/control/command.hpp"

#include "subsystems/gimbal/gimbal_subsystem.hpp"
#include "drivers.hpp"

#if defined (TARGET_STANDARD)
#include "controls/standard/standard_constants.hpp"
#elif defined (TARGET_SENTRY)
#include "controls/sentry/sentry_constants.hpp"
#elif defined (TARGET_HERO)
#include "controls/hero/hero_constants.hpp"
#endif

namespace gimbal{
    /**
     * @brief Command that will allow for user input as well as a correction to gimbal beyblading
     * 
     */
class GimbalBeybladeCommand : public tap::control::Command {
public:

/**
 * @brief Constructor for gimbal beyblade, exact same to other gimbal commands
 */
    GimbalBeybladeCommand(GimbalSubsystem *const gimbal, src::Drivers *drivers);

    //should never copy
    GimbalBeybladeCommand(const GimbalBeybladeCommand &other) = delete;

    GimbalBeybladeCommand &operator=(const GimbalBeybladeCommand &other) = delete;

    /**
     * @brief Sets the gimbal to be on beyblade Mode and raises pitch
     * 
     * @return ** void 
     */
    void initialize() override;

    const char *getName() const { return "gimbal beyblade command"; }

    /**
     * @brief Continously called, receives and passes on gimbal remote control inputs
     * 
     * @return ** void 
     */
    void execute() override;

    /**
     * @brief Called when finished, turns off beyblade mode of gimbal and end inputs
     * 
     * @return ** void 
     */
    void end(bool) override;

    bool isFinished() const override;
private:
    GimbalSubsystem* gimbal;
    src::Drivers* drivers;
    bool noTurn;
    float offset = 1.7;
    float rotation = (BEYBLADE_INPUT / offset);

}; //GimbalBeybladeCommand


}// namespace gimbal

#endif