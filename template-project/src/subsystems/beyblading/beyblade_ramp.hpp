#ifndef BEYBLADE_RAMP_HPP_
#define BEYBLADE_RAMP_HPP_

#include "modm/math/filter/ramp.hpp"

class BeybladeRamp{
public:
private:
    modm::filter::Ramp<float> beybladeRamp;
}; //BeybladeRamp

#endif