#ifdef TARGET_STANDARD
#include "drivers.hpp"
#include "drivers_singleton.hpp"

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "subsystems/chassis/chassis_subsystem.hpp"
#include "subsystems/gimbal/gimbal_subsystem.hpp"
#include "subsystems/feeder/feeder_subsystem.hpp"
#include "subsystems/shooter/shooter_subsystem.hpp"

#include "subsystems/shooter/shoot_user_command.hpp"
#include "subsystems/chassis/chassis_movement_command.hpp"
#include "subsystems/gimbal/gimbal_movement_command.hpp"
#include "subsystems/music/music_player.hpp"
#include "subsystems/gimbal/gimbal_motor_interface.hpp"
#include "subsystems/feeder/feeder_movement_command.hpp"


src::driversFunc drivers = src::DoNotUse_getDrivers;

using namespace tap;
using namespace tap::control;
using namespace tap::communication::serial;
using namespace chassis;
using namespace gimbal;
using namespace music;
using namespace feeder;
using namespace shooter;

namespace src::control{
// Define subsystems here ------------------------------------------------
ChassisSubsystem chassis(drivers());
GimbalSubsystem gimbal(drivers());
FeederSubsystem feeder(drivers());
ShooterSubsystem shooter(drivers());
// Robot Specific Controllers ------------------------------------------------
MusicPlayer sound_track(drivers(), PIANO_MAN, PIANO_MAN_BPM);
GimbalInterface gimbalInterface(&gimbal);

// Define commands here ---------------------------------------------------
ChassisMovementCommand chassisMovement(&chassis, drivers(), &gimbalInterface);
GimbalMovementCommand gimbalMovement(&gimbal, drivers());
FeederMovementCommand feederMovement(&feeder, drivers());
ShootUserCommand shootUser(&shooter, drivers());

// Define command mappings here -------------------------------------------
HoldCommandMapping rightSwitchMid(drivers(), {&chassisMovement, &gimbalMovement}, 
RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

HoldCommandMapping rightSwitchUp(drivers(), {&gimbalMovement}, 
RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

HoldCommandMapping leftSwitchDown(drivers(), {&feederMovement},
RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));

HoldCommandMapping leftSwitchUp(drivers(), {&shootUser}, 
RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));
// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers){
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&gimbal);
    drivers->commandScheduler.registerSubsystem(&feeder);
    drivers->commandScheduler.registerSubsystem(&shooter);
}
// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    //sound_track.execute();
    chassis.initialize();
    gimbal.initialize();
    feeder.initialize();
    shooter.initialize();
}
// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers* drivers) {
    //chassis.setDefaultCommand(&chassisMovement);
}
// Set Commands scheduled on startup
void startupCommands(src::Drivers* drivers) {

}
// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers* drivers) {
    drivers->commandMapper.addMap(&rightSwitchMid);
    //drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchUp);
}
}//namespace src::control


// Initialize subsystems ---------------------------------------------------
namespace src::control
{
    void initializeSubsystemCommands(src::Drivers* drivers)
    {
        initializeSubsystems();
        registerSubsystems(drivers);
        setDefaultCommands(drivers);
        startupCommands(drivers);
        registerIOMappings(drivers);
    }
} //namespace src::Control

#endif