#ifndef REF_INTERFACE_HPP_
#define REF_INTERFACE_HPP_

#include "tap/communication/serial/ref_serial.hpp"
#include "cv_com_sending_structs.hpp"


namespace src
{
class Drivers;
}
/**
 * @brief RefInterface is an interface for retreiving specific referee data.
 * 
 */
class RefInterface{
public:
    RefInterface(src::Drivers * drivers) : drivers(drivers) {}

    void updateData();


    cv::RefStructObj getData() const {return returnData;}

    //first: power usage, second: power limit
    std::pair<uint16_t, uint16_t> getPowerUsage() const{
        return std::pair<uint16_t, uint16_t>(powerUsage, powerLimit);
    }

    #if defined (TARGET_HERO)
    //first = current heat, second = heat limit
    std::pair<uint16_t, uint16_t> getShooterHeat() const{
        return std::pair<uint16_t, uint16_t>(heat42, heatLimit42);
    }
    #elif defined (TARGET_STANDARD)
    //first = current heat, second = heat limit
    std::pair<uint16_t, uint16_t> getShooterHeat() const{
        return std::pair<uint16_t, uint16_t>(heat17ID1, heatLimit17ID1);
    }
    #elif defined (TARGET_SENTRY)
    //0 = current heat 1, 1 = heat limit 1, 2 = current heat 2, 3 = heat limit 2,
    std::vector<uint16_t> getShooterHeat() const{
        std::vector<uint16_t> vec{heat17ID1, heatLimit17ID1, heat17ID2, heatLimit17ID2};
        return vec;
    }
    #endif

    uint8_t getLevel() const {return returnData.robotLevel;}

    uint16_t getHp() const {return returnData.currHP;}

    bool refDataValid() const {return returnData.maxHP;}

    bool getShooterPowerStatus() const {return shooterPowerStatus;}

    bool getEnemyColor() const{return returnData.robotID < 10;}

    bool gameStarted() const {return returnData.stage == 4;}

    bool gameFinished() const {return returnData.stage == 5;}

    //bool getShooterPowerStatus() const{}
private:
    src::Drivers * drivers;
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

    bool shooterPowerStatus;         ///< shooter power status true: has power false: no power
    
    cv::RefStructObj returnData;

}; //RefInterface
#endif