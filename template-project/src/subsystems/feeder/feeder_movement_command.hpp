#ifndef FEEDER_MOVEMENT_COMMAND_HPP_
#define FEEDER_MOVEMENT_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "feeder_subsystem.hpp"
#include "drivers.hpp"
#include "subsystems/shooter/shooter_Interface.hpp"
namespace feeder{

class FeederMovementCommand : public tap::control::Command{
public:
    /**
     * Initializes the command with the passed in FeederSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] feeder a pointer to the feeder motorto be passed in that this
     *      Command will interact with.
     */
    FeederMovementCommand(FeederSubsystem *const feeder, src::Drivers *drivers, shooter::ShooterInterface* shoot);

    FeederMovementCommand(const FeederMovementCommand &other) = delete;

    FeederMovementCommand &operator=(const FeederMovementCommand &other) = delete;

    //sets the feeder to move to target rpm based on robot level
    void initialize() override;

    const char *getName() const { return "feeder drive command"; }

    //checks if about to reach heat limit, if so, turn off feeder
    void execute() override;

    //turns off feeder
    void end(bool) override;

    bool isFinished() const override;

    //returns if you are about to hit the heat limit or not
    bool checkBarrelHeatLimit() const;
private:
    FeederSubsystem *const feeder;
    tap::arch::MilliTimeout burstFireTimeout;
    src::Drivers *drivers;
    shooter::ShooterInterface* shooter;

}; //class FeederMovementCommand : public tap::control::Command

} //namespace Feeder

#endif //FEEDER_MOVEMENT_COMMAND_HPP_