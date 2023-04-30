#ifndef REF_INTERFACE_HPP_
#define REF_INTERFACE_HPP_

#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/util_macros.hpp"
#include "tap/communication/serial/ref_serial.hpp"

namespace src
{
class Drivers;
}

class RefInterface{
public:
    RefInterface(src::Drivers * drivers) : drivers(drivers) {}

    void updateData();

    unsigned int getHP() const{ return currHP; }
private:
    unsigned int currHP;
    src::Drivers * drivers;
    unsigned char robotID;
};

#endif