#ifndef SHOOTER_SUBSYSTEM_HPP_
#define SHOOTER_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"

#if defined (TARGET_STANDARD)
#include "controls/standard/standard_constants.hpp"
#elif defined (TARGET_SENTRY)
#include "controls/sentry/sentry_constants.hpp"
#elif defined (TARGET_HERO)
#include "controls/hero/hero_constants.hpp"
#endif

#include "tap/communication/gpio/pwm.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/drivers.hpp"
#include "tap/algorithms/ramp.hpp"
#include "modm/math/filter/ramp.hpp"
#include "tap/motor/dji_motor.hpp"

namespace shooter
{
/**
 * A bare bones subsystem to control PWM flywheels
 */
class ShooterSubsystem : public tap::control::Subsystem
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
     * Constructs a new shooterSubsystem with default parameters specified in
     * the private section of this class.
     */
#if defined (TARGET_STANDARD) || defined (TARGET_SENTRY)
    ShooterSubsystem(tap::Drivers *drivers)
        : tap::control::Subsystem(drivers),
        flywheelRamp(0.05f, 0.05f, 0.15f)
    {
    }
#elif defined (TARGET_HERO)
ShooterSubsystem(tap::Drivers *drivers)
        : tap::control::Subsystem(drivers),
        flywheelRamp(10, 10, 0),
         flywheel1(drivers,
               tap::motor::MOTOR2,
               tap::can::CanBus::CAN_BUS2,
               true,
               "Flywheel 1"),
        flywheel2(drivers,
               tap::motor::MOTOR4,
               tap::can::CanBus::CAN_BUS2,
               false,
               "Flywheel 2"),
        pid1(pidVals.PID_KP, pidVals.PID_KI, pidVals.PID_KD,
      pidVals.PID_MAX_IOUT, pidVals.PID_MAX_OUT),
      pid2(pidVals.PID_KP, pidVals.PID_KI, pidVals.PID_KD,
      pidVals.PID_MAX_IOUT, pidVals.PID_MAX_OUT)
    {
    }


//updates pid controller based off of current rpm
void ShooterSubsystem::updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRpm);
#endif

    ShooterSubsystem(const ShooterSubsystem &other) = delete;

    ShooterSubsystem &operator=(const ShooterSubsystem &other) = delete;

    ~ShooterSubsystem() = default;

    void initialize() override;

    /**
     * No-op function that is a placeholder because all interactions with motors are done
     * in setDesiredOutput.
     */
    void refresh() override;

    void setDesiredOutput(float output);

    float findRampOutput(float output);

    void changeOnFlag(){on = !on;}

private:
#if defined (TARGET_STANDARD)
    ///pwm ports on dev board
    tap::gpio::Pwm::Pin flywheel1 = tap::gpio::Pwm::W;
    tap::gpio::Pwm::Pin flywheel2 = tap::gpio::Pwm::X;
#elif defined (TARGET_SENTRY)
    tap::gpio::Pwm::Pin flywheel1 = tap::gpio::Pwm::W;
    tap::gpio::Pwm::Pin flywheel2 = tap::gpio::Pwm::X;
    tap::gpio::Pwm::Pin flywheel3 = tap::gpio::Pwm::Y;
    tap::gpio::Pwm::Pin flywheel4 = tap::gpio::Pwm::Z;
#elif defined (TARGET_HERO)
        // PID controllers for RPM feedback from wheels
    modm::filter::Ramp<int> flywheelRamp;
    ///< Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor flywheel1;
    tap::motor::DjiMotor flywheel2;

    modm::Pid<float> pid1;
    modm::Pid<float> pid2;

    SHOOTER_PID pidVals;

    float targetRPM = 0;
#endif
    // PID controllers for RPM feedback from wheels
    modm::filter::Ramp<float> flywheelRamp;

    bool online = false;

    bool on = false;

    ///< Any user input is translated into desired RPM for each motor.

    // Scale factor for converting joystick movement into RPM setpoint
    //static constexpr float RPM_SCALE_FACTOR = 4000.0f;

};  // class ShooterSubsystem

}  // namespace shooter


#endif  // SHOOTER_SUBSYSTEM_HPP_