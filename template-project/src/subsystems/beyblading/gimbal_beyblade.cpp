#include "gimbal_beyblade.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"


namespace gimbal
{
GimbalBeybladeCommand::GimbalBeybladeCommand(GimbalSubsystem *const gimbal, src::Drivers *drivers)
: gimbal(gimbal), drivers(drivers)
{
     if (gimbal == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(gimbal));
}
void  GimbalBeybladeCommand::initialize() {
        gimbal->allignGimbal();
        noTurn = 0;
        gimbal->setBeybladeMode(1);
    }

void  GimbalBeybladeCommand::execute()
{
    //gimbal->setBeybladeMode(drivers->cv_com.getBeybladeMode());
    gimbal->controllerInput(drivers->control_interface.getGimbalYawInput(),
        drivers->control_interface.getGimbalPitchInput());
}

void  GimbalBeybladeCommand::end(bool) {
    gimbal->controllerInput(0, 0);
    gimbal->noInputs();
    gimbal->setBeybladeMode(0);
    }

bool  GimbalBeybladeCommand::isFinished() const { return false; }

}//namespace gimbal