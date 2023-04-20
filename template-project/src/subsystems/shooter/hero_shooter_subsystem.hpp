#ifndef HERO_SHOOTER_SUBSYSTEM_HPP_
#define HERO_SHOOTER_SUBSYSTEM_HPP_
#ifdef TARGET_HERO

#include "tap/control/subsystem.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"

#include "controls/hero/hero_constants.hpp"

#include "tap/communication/gpio/pwm.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/drivers.hpp"
#include "tap/algorithms/ramp.hpp"
#include "modm/math/filter/ramp.hpp"
#include "modm/math/filter/pid.hpp"

namespace shooter
{
/**
 * A bare bones subsystem to control PWM flywheels
 */
class HeroShooterSubsystem : public tap::control::Subsystem
{
public:
    /**
     * This max output is measured in the c620 robomaster translated current.
     * Per the datasheet, the controllable current range is -16384 ~ 0 ~ 16384.
     * The corresponding speed controller output torque current range is
     * -20 ~ 0 ~ 20 A.
     *
     * For this demo, we have capped the output at 8000. This should be more
     * than enough for what you are doing.
     */
    static constexpr float MAX_CURRENT_OUTPUT = 8000.0f;

    /**
     * Constructs a new HeroShooterSubsystem with default parameters specified in
     * the private section of this class.
     */
    HeroShooterSubsystem(tap::Drivers *drivers)
        : tap::control::Subsystem(drivers),
        flywheelRamp(10, 10, 0),
         flywheel1(drivers,
               tap::motor::MOTOR2,
               tap::can::CanBus::CAN_BUS2,
               false,
               "Flywheel 1"),
        flywheel2(drivers,
               tap::motor::MOTOR3,
               tap::can::CanBus::CAN_BUS2,
               true,
               "Flywheel 2"),
        pid1(pidVals.PID_KP, pidVals.PID_KI, pidVals.PID_KD,
      pidVals.PID_MAX_IOUT, pidVals.PID_MAX_OUT),
      pid2(pidVals.PID_KP, pidVals.PID_KI, pidVals.PID_KD,
      pidVals.PID_MAX_IOUT, pidVals.PID_MAX_OUT)
    {
    }

    HeroShooterSubsystem(const HeroShooterSubsystem &other) = delete;

    HeroShooterSubsystem &operator=(const HeroShooterSubsystem &other) = delete;

    ~HeroShooterSubsystem() = default;

    void initialize() override;

    /**
     * No-op function that is a placeholder because all interactions with motors are done
     * in setDesiredOutput.
     */
    void refresh() override;

    void setDesiredOutput(float output);

    float findRampOutput(float output);

    void updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRpm);

    bool flywheel1Online() const {return flywheel1.isMotorOnline();}
    bool flywheel2Online() const {return flywheel2.isMotorOnline();}

    //void updateRpmPid(tap::algorithms::Ramp* ramp, tap::gpio::Pwm::Pin const flywheel);

private:
    // PID controllers for RPM feedback from wheels
    modm::filter::Ramp<int> flywheelRamp;
    ///< Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor flywheel1;
    tap::motor::DjiMotor flywheel2;

    modm::Pid<float> pid1;
    modm::Pid<float> pid2;

    SHOOTER_PID pidVals;

    bool online = false;

    float targetRPM = 0;

    ///< Any user input is translated into desired RPM for each motor.

    // Scale factor for converting joystick movement into RPM setpoint
    //static constexpr float RPM_SCALE_FACTOR = 4000.0f;

};  // class HeroShooterSubsystem

}  // namespace shooter

#endif
#endif  // SHOOTER_SUBSYSTEM_HPP_