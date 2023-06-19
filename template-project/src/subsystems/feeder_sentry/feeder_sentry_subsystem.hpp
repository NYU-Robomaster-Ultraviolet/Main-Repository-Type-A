#ifdef TARGET_SENTRY

#ifndef FEEDER_SENTRY_SUBSYSTEM_HPP_ 
#define FEEDER_SENTRY_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"

#include "controls/sentry/sentry_constants.hpp"

#include "drivers.hpp"

namespace feeder{

class FeederSentrySubsystem : public tap::control::Subsystem
{
public:
    FeederSentrySubsystem(tap::Drivers *drivers)
    : tap::control::Subsystem(drivers),
    feederMotor1(drivers,
               tap::motor::MOTOR7,
               tap::can::CanBus::CAN_BUS2,
               false,
               "Feeder Motor 1"),
    feederMotor2(drivers,
            tap::motor::MOTOR8, // either 6 or 8
            tap::can::CanBus::CAN_BUS2,
            true,
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

    void initialize() override;
    void refresh() override;

    const char* getName() override {return "feeder subsystem";}

    void setTargetRPM(float RPM);

    float getFeederMotorRPM() const {return feederMotor1.isMotorOnline() ? feederMotor1.getShaftRPM() : 0.0f; }

    float getFeederMotor2RPM() const {return feederMotor2.isMotorOnline() ? feederMotor2.getShaftRPM() : 0.0f; }

    void updateFeederPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRpm);

    bool motor1Online() { return feederMotor1.isMotorOnline(); }

    bool motor2Online() { return feederMotor2.isMotorOnline(); }

private:
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

    //PID constants
    FEEDER_PID feederPid;

    bool inputsFound = false;

};  //class FeederSentrySubsystem

}   //namespace feeder

#endif
#endif
