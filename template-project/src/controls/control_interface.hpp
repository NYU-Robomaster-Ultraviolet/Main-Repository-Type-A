#ifndef CONTROL_INTERFACE_HPP_
#define CONTROL_INTERFACE_HPP_

#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/util_macros.hpp"
#include "tap/architecture/timeout.hpp"

using namespace tap :: algorithms;

namespace tap{
    class Drivers;
}

namespace src::control {

class ControlInterface{
    private:
        tap::Drivers *drivers;

        //stored counters, used to prevent rereading signals
        uint32_t prevUpdateCounterX = 0;
        uint32_t prevUpdateCounterY = 0;
        uint32_t prevUpdateCounterRotation = 0;

        //sensitivity for inputs
        uint32_t X_SENSITIVITY = 3.96f; //Max Input of 660 multiplied by sensitivity of .006
        uint32_t Y_SENSITIVITY = 3.3f; //Max Input of 660 multiplied by sensitivity of .005

        LinearInterpolationPredictor chassisXInput;
        LinearInterpolationPredictor chassisYInput;
        LinearInterpolationPredictor chassisRotationInput;

        tap::arch::MilliTimeout shiftCheckTimeout;
        tap::arch::MilliTimeout ctrChecKTimeout;

        bool shiftMode = false;
        bool ctrMode = false;

    public:
        ControlInterface(tap::Drivers *drivers) : drivers(drivers) {}

        //gets the inputs from keyboard and mouse
        mockable float getChassisXInput();
        mockable float getChassisYInput();
        mockable float getChassisRotationInput();

        mockable float getGimbalYawInput();
        mockable float getGimbalPitchInput();

        //initializes timeouts for checking key presses
        void init();

        //will check key presses for ctr and shift, set flags if read then delay the mode change by a second
        void checkKeyPresses();
        
};


}

#endif  // CONTROL_INTERFACE_HPP_