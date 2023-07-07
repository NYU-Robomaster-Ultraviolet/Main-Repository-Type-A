#ifndef SHOOTER_INTERFACE_HPP_
#define SHOOTER_INTERFACE_HPP_

#include "shooter_subsystem.hpp"
namespace shooter{
class ShooterInterface{
public:
    ShooterInterface(ShooterSubsystem* shoot) : shooter(shoot){}
    bool flywheelsOn() const {return shooter->getOnFlag();}
private:
ShooterSubsystem* shooter;
};//ShooterInterface
}//shooter
#endif