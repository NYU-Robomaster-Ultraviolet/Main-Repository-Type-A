#ifndef FEEDER_SUBSYSTEM_HPP_
#define FEEDER_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "controls/standard/standard_constants.hpp"
#include "drivers.hpp"

namespace feeder{

class FeederSubsystem : public tap::control::Subsystem
{
public:
    FeederSubsystem(tap::Drivers *drivers)
    : tap::control::Subsystem(drivers), 
    feederMotor1(drivers,
               tap::motor::MOTOR7,
               tap::can::CanBus::CAN_BUS2,
               false,
               "Feeder Motor 1"),
    feederMotor2(drivers,
               tap::motor::MOTOR8, // placeholder
               tap::can::CanBus::CAN_BUS2,
               false,
               "Feeder Motor 2"),
      targetRPM(0.0f),
      currentFeederMotorSpeed(0.0f),
      rpmPid(feederPid.PID_KP, feederPid.PID_KI, feederPid.PID_KD, 
      feederPid.PID_MAX_IOUT, feederPid.PID_MAX_OUT)
      {}

    void initialize() override;
    void refresh() override;

    const char* getName() override {return "feeder two motor subsystem";}

    void setTargetRPM(float RPM);

    float getFeederMotorRPM() const {return feederMotor1.isMotorOnline() && feederMotor2.isMotorOnline() ? (feederMotor1.getShaftRPM() + feederMotor2.getShaftRPM()) / 2 : 0.0f; }

    float getFeederMotor1RPM() const {return feederMotor1.isMotorOnline() ? feederMotor1.getShaftRPM() : 0.0f; }

    float getFeederMotor2RPM() const {return feederMotor2.isMotorOnline() ? feederMotor2.getShaftRPM() : 0.0f; }


    void updateFeederPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor1, tap::motor::DjiMotor* const motor2, float desiredRpm);

    bool motorOnline(){ return feederMotor1.isMotorOnline() && feederMotor2.isMotorOnline();
    }

private:
    tap::motor::DjiMotor feederMotor1;
    tap::motor::DjiMotor feederMotor2;

    //target RPM, idk the range of its value
    float targetRPM;

    //motor speed given in revolutions / min
    float currentFeederMotorSpeed;

    modm::Pid<float> rpmPid;

    //PID constants
    FEEDER_PID feederPid;

    bool inputsFound = false;

};  //class FeederSubsystem

}   //namespace feeder

#endif