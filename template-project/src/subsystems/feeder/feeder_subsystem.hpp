#ifndef FEEDER_SUBSYSTEM_HPP_
#define FEEDER_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"

#if defined (TARGET_STANDARD)
#include "controls/standard/standard_constants.hpp"
#elif defined (TARGET_HERO)
#include "controls/hero/hero_constants.hpp"
#elif defined (TARGET_SENTRY)
#include "controls/sentry/sentry_constants.hpp"
#endif

#include "drivers.hpp"

namespace feeder{

class FeederSubsystem : public tap::control::Subsystem
{
public:

#if defined(TARGET_SENTRY)
FeederSubsystem(tap::Drivers *drivers)
    : tap::control::Subsystem(drivers),
    feederMotor1(drivers,
               constants.FEEDER_MOTOR_ID,
               constants.CAN_BUS,
               constants.FEEDER_REVERSED,
               "Feeder Motor 1"),
    feederMotor2(drivers,
            constants.FEEDER_MOTOR_ID2, // either 6 or 8
            constants.CAN_BUS,
            constants.FEEDER_REVERSED_2,
            "Feeder Motor 2"),
      motor1TargetRPM(0.0f),
      motor2TargetRPM(0.0f),
      currentFeederMotor1Speed(0.0f),
      currentFeederMotor2Speed(0.0f),
      rpmPid1(feederPid.PID_KP, feederPid.PID_KI, feederPid.PID_KD,
        feederPid.PID_MAX_IOUT, feederPid.PID_MAX_OUT),
      rpmPid2(feederPid.PID_KP, feederPid.PID_KI, feederPid.PID_KD,
        feederPid.PID_MAX_IOUT, feederPid.PID_MAX_OUT)
      {}
#elif defined (TARGET_HERO) || defined (TARGET_STANDARD)
    FeederSubsystem(tap::Drivers *drivers)
    : tap::control::Subsystem(drivers),
    feederMotor(drivers,
               constants.FEEDER_MOTOR_ID,
               constants.CAN_BUS,
               constants.FEEDER_REVERSED,
               "Feeder Motor"),
      targetRPM(0.0f),
      currentFeederMotorSpeed(0.0f),
      rpmPid(feederPid.PID_KP, feederPid.PID_KI, feederPid.PID_KD,
      feederPid.PID_MAX_IOUT, feederPid.PID_MAX_OUT)
      {}
#endif

    void initialize() override;
    void refresh() override;

    const char* getName() override {return "feeder subsystem";}

    void setTargetRPM(float RPM);

    void updateFeederPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRpm);

    #if defined (TARGET_SENTRY)
    float getFeederMotorRPM() const {return feederMotor1.isMotorOnline() ? feederMotor1.getShaftRPM() : 0.0f; }

    float getFeederMotor2RPM() const {return feederMotor2.isMotorOnline() ? feederMotor2.getShaftRPM() : 0.0f; }

    bool motor1Online() { return feederMotor1.isMotorOnline(); }

    bool motor2Online() { return feederMotor2.isMotorOnline(); }

    #elif defined (TARGET_HERO) || defined (TARGET_STANDARD)
    bool motorOnline(){ return feederMotor.isMotorOnline();
    }

     float getFeederMotorRPM() const {return feederMotor.isMotorOnline() ? feederMotor.getShaftRPM() : 0.0f; }
     #endif

private:

    #if defined (TARGET_SENTRY)
    tap::motor::DjiMotor feederMotor1;
    tap::motor::DjiMotor feederMotor2;

    //target RPM, idk the range of its value
    float motor1TargetRPM;
    float motor2TargetRPM;

    //motor speed given in revolutions / min
    float currentFeederMotor1Speed;
    float currentFeederMotor2Speed;

    modm::Pid<float> rpmPid1;
    modm::Pid<float> rpmPid2;
    #elif defined (TARGET_STANDARD) || defined (TARGET_HERO)
    tap::motor::DjiMotor feederMotor;

    //target RPM, idk the range of its value
    float targetRPM;

    //motor speed given in revolutions / min
    float currentFeederMotorSpeed;

    modm::Pid<float> rpmPid;
    #endif

    //PID constants
    FEEDER_PID feederPid;

    //feeder constants
    Feeder_CONSTANTS constants;

};  //class FeederSubsystem

}   //namespace feeder

#endif