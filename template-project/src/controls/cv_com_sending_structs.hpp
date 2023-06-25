#ifndef CV_COM_SENDING_STRUCTS_HPP_
#define CV_COM_SENDING_STRUCTS_HPP_
namespace cv{

//sends current yaw and pitch
typedef struct enableStruct
    {
        unsigned char header = 0xE7;
        unsigned short length = 1;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type = 1;
        bool enable;
        char footer = 0;
    } enableStructObj, *EnableStruct;

//states which color team
typedef struct colorStruct
    {
        unsigned char header = 0xE7;
        unsigned short length = 1;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type = 2;
        unsigned char color;
        char footer = 0;
    } ColorStructObj, *ColorStruct;

//sends current yaw and pitch
typedef struct sendingAngleStruct
    {
        unsigned char header = 0xE7;
        unsigned short length = 8;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type = 5;
        int pitch;
        int yaw;
        char footer = 0;
    } SendingAngleStructObj, *SendingAngleStruct;

typedef struct RefereeCVData{
        unsigned char header = 0xE7;
        unsigned short length = 54; //54
        unsigned char empty = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type = 7;
        //general robot data
        uint16_t robotID;   //Robot ID 1-9 for red, 101-109 for blue
        uint16_t maxHP;     ///current HP of robot
        uint16_t currHP;     ///current HP of robot
        uint16_t robotLevel;       ///< Current level of this robot (1-3).

        //time and stage
        uint16_t stage; //stage of the game
            // PREMATCH = 0,        ///< Pre-competition. stage
            // SETUP = 1,           ///< Setup stage.
            // INITIALIZATION = 2,  ///< Initialization stage.
            // COUNTDOWN = 3,       ///< 5-second countdown.
            // IN_GAME = 4,         ///< In middle of the game.
            // END_GAME = 5,        ///< Calculating competition results.

        uint16_t timeLeft; //time left in the round

        //All robots HP
        //redHP
        uint16_t redHero1;
        // uint16_t redEngineer2;
        uint16_t redStandard3;
        uint16_t redStandard4;
        uint16_t redStandard5;
        uint16_t redSentry7;
        //uint16_t redOutpost;
        uint16_t redBase;
        //blueHP
        uint16_t blueHero1;
        // uint16_t blueEngineer2;
        uint16_t blueStandard3;
        uint16_t blueStandard4;
        uint16_t blueStandard5;
        uint16_t blueSentry7;
        //uint16_t blueOutpost;
        uint16_t blueBase;

        //uint16_t remainingCoins;
        //RFID info (all bools)
        uint16_t rfid;           //0: Robot in the base zone
                                //1: Robot in the elevated ground zone
                                //2: Robot in rune game activation zone
                                //3: Robot in launch ramp zone (section before the actual ramp)
                                //4: Robot adjacent to the outpost
                                //5: Robot adjacent to the resource island
                                //6: Robot in restoration zone
                                //7: Engineer's RFID swipe card is beneath RFID card and is activating the card

        //chassis data power usage
        // uint16_t powerUsage;       //rounded don in W
        // uint16_t powerLimit;

        //shooting data
        // uint16_t heat17ID1;              ///< Current 17mm turret heat, ID2.
        // uint16_t heat17ID2;              ///< ID2 turret heat.
        // uint16_t heat42;                 ///< Current 42mm turret heat.

        // uint16_t heatLimit17ID1;         ///< 17mm turret heat limit, ID1.
        // uint16_t heatLimit17ID2;         ///< ID2.
        // uint16_t heatLimit42;            ///< 42mm turret heat limit.
        
         //armor plates + taken damage
        uint16_t damagedArmorId;   ///< Armor ID that was damaged
        uint16_t damageType; ///< Armor damage. ARMOR_DAMAGE = 0,           ///< Armor damage.
                        // MODULE_OFFLINE = 1,         ///< Module offline.
                        // BARREL_OVER_SPEED = 2,      ///< Firing speed too high.
                        // BARREL_OVERHEAT = 3,        ///< Barrel overheat.
                        // CHASSIS_POWER_OVERRUN = 4,  ///< Chassis power overrun.
                        // COLLISION = 5,              ///< Armor plate collision.
        

        //referee warnings
        uint16_t warningLevel; //0: none 1: yellow card 2: red card 3: forfeture 
        uint16_t warningRobotID;   //Robot ID 1-9 for red, 101-109 for blue

        uint32_t receivedDpms; //damage taken per min
        uint32_t lastReceivedWarningRobotTime;  ///< Last time (in milliseconds) that a warning was received.

        char footer;
    } RefStructObj, *RefStruct;//refereeCVData
};// namespace CV
#endif