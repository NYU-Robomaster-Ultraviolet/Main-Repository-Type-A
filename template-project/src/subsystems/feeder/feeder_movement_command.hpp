#ifndef FEEDER_MOVEMENT_COMMAND_HPP_
#define FEEDER_MOVEMENT_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "feeder_subsystem.hpp"
#include "drivers.hpp"
namespace feeder{

class FeederMovementCommand : public tap::control::Command{
public:
    /**
     * Initializes the command with the passed in FeederSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] feeder a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    FeederMovementCommand(FeederSubsystem *const feeder, src::Drivers *drivers);

    FeederMovementCommand(const FeederMovementCommand &other) = delete;

    FeederMovementCommand &operator=(const FeederMovementCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "feeder drive command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;
private:
    FeederSubsystem *const feeder;

    src::Drivers *drivers;
}; //class FeederMovementCommand : public tap::control::Command

} //namespace Feeder

#endif //FEEDER_MOVEMENT_COMMAND_HPP_ 