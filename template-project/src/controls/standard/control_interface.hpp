#ifndef CONTROL_INTERFACE_HPP_
#define CONTROL_INTERFACE_HPP_

#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/util_macros.hpp"

using namespace tap :: algorithms;

namespace tap{
    class Drivers;
}

namespace src::control {

class ControlInterface{
    private:
        tap::Drivers *drivers;

        uint32_t prevUpdateCounterX = 0;
        uint32_t prevUpdateCounterY = 0;
        uint32_t prevUpdateCounterRotation = 0;

        uint32_t X_SENSITIVITY = 3.96f; //Max Input of 660 multiplied by sensitivity of .006
        uint32_t Y_SENSITIVITY = 3.3f; //Max Input of 660 multiplied by sensitivity of .005

        LinearInterpolationPredictor chassisXInput;
        LinearInterpolationPredictor chassisYInput;
        LinearInterpolationPredictor chassisRotationInput;
    public:
        ControlInterface(tap::Drivers *drivers) : drivers(drivers) {}

        //final inputs 
        mockable float getChassisXInput();
        mockable float getChassisYInput();
        mockable float getChassisRotationInput();

        mockable float getGimbalYawInput();
        mockable float getGimbalPitchInput();
};


}

#endif  // CONTROL_INTERFACE_HPP_