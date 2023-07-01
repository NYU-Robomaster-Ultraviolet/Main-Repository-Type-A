#ifndef REF_INTERFACE_HPP_
#define REF_INTERFACE_HPP_

#include "tap/communication/serial/ref_serial.hpp"
#include "cv_com_sending_structs.hpp"


namespace src
{
class Drivers;
}

class RefInterface{
public:
    RefInterface(src::Drivers * drivers) : drivers(drivers) {}

    void updateData();

    


    cv::RefStructObj getData() const {return returnData;}

    //power usage, power limit
    std::pair<uint16_t, uint16_t> getPowerUsage() const{
        return std::pair<uint16_t, uint16_t>(powerUsage, powerLimit);
    }

    #ifdef TARGET_HERO
    std::pair<uint16_t, uint16_t> getShooterHeat() const{
        return std::pair<uint16_t, uint16_t>(heat42, heat42);
    }
    bool nearPowerLimit() const{
        std::pair<uint16_t, uint16_t> heat = getShooterHeat();
        
    }
    #endif
    #ifdef TARGET_STANDARD
    std::pair<uint16_t, uint16_t> getShooterHeat() const{
        return std::pair<uint16_t, uint16_t>(heat17ID1, heatLimit17ID1);
    }
    #endif
    #ifdef TARGET_SENTRY
    std::vector<uint16_t> getShooterHeat() const{
        std::vector<uint16_t> vec{heat17ID1, heat17ID2, heatLimit17ID1, heatLimit17ID2};
        return vec;
    }
    #endif
private:

//chassis data power usage
        uint16_t powerUsage;       //rounded don in W
        uint16_t powerLimit;

        //shooting data
        uint16_t heat17ID1;              ///< Current 17mm turret heat, ID2.
        uint16_t heat17ID2;              ///< ID2 turret heat.
        uint16_t heat42;                 ///< Current 42mm turret heat.

        uint16_t heatLimit17ID1;         ///< 17mm turret heat limit, ID1.
        uint16_t heatLimit17ID2;         ///< ID2.
        uint16_t heatLimit42;            ///< 42mm turret heat limit.
    src::Drivers * drivers;
    cv::RefStructObj returnData;

}; //RefInterface
#endif