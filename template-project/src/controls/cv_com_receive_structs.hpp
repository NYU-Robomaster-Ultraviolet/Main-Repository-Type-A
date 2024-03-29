#ifndef CV_COM_RECEIVE_STRUCTS_HPP_
#define CV_COM_RECEIVE_STRUCTS_HPP_
namespace cv{
//message type: 1 receive message about movement and targeting
typedef struct autoAimStruct
    {
        unsigned char header = 0xE7;
        unsigned short length = 9;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type = 1;
        int pitch;
        int yaw;
        unsigned char hasTarget;
        unsigned short footer = 0;
    } AutoAimStructObj, *AutoAimStruct;

// message_type: 2 chassis movement structs
typedef struct chassisMoveStruct
    {
        unsigned char header = 0xE7;
        unsigned short length;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type = 2;  // 2
        // value should be between -1000 and 1000;
        short chassisX;  // chassis x axis movement
        short chassisY;  // chassis y axis movement
        short chassisR;  // rotational movement
        // 0 : default movement
        unsigned char mode;  // used to switch command behavior
        unsigned short footer = 0;
    } ChassisMoveStructObj, *ChassisMoveStruct;

// message_type: 3
typedef struct alignFinishStruct
    {
        unsigned char header = 0xE7;
        unsigned short length = 1;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type = 3;
        unsigned char x;
        unsigned short footer = 0;
    } AlignFinishStructObj, *AlignFinishStruct;

    // message_type: 4
typedef struct gimbalMoveStruct
    {
        unsigned char header = 0xE7;
        unsigned short length;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type = 4;  // 4
        // value should be between -1000 and 1000;
        short gimbalX;  // gimbal yaw movement
        short gimbalY;  // gimbal pitch movement
        // 0 : default movement, 1 : beyblade, 2 : beyblade backwards
        unsigned char mode;  // used to switch command behavior
        unsigned short footer = 0;
    } GimbalMoveStructObj, *GimbalMoveStruct;

//straight foward movement
typedef struct MoveStraight
    {
        unsigned char header = 0xE7;
        unsigned short length = 8;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type = 5;
        unsigned short distance; //in mm
        short velocity; //m/s, divide by 1000 to get
        unsigned short footer = 0;
    } MoveStraightObj, *MoveStraightStruct;

//straight foward movement
typedef struct SpinChassis
    {
        unsigned char header = 0xE7;
        unsigned short length = 8;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type = 6;
        short angle; //in degrees
        short velocity; //degree/s, divide by 1000 to get
        unsigned short footer = 0;
    } SpinChassisObj, *SpinChassisStruct;

//seting power given
typedef struct SetPower
    {
        unsigned char header = 0xE7;
        unsigned short length = 16;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type = 7;
        short xLinearPower;
        short yLinearPower;
        short xAnglePower;
        short yAnglePower;
        unsigned short footer = 0;
    } SetPowerObj, *SetPowerStruct;

//set the velocity of the gimbal and chassis
typedef struct SetVelocity
    {
        unsigned char header = 0xE7;
        unsigned short length = 16;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type = 8;
        short xVelocity;
        short yVelocity;
        short yawVelocity;
        short pitchVelocity;
        unsigned short footer = 0;
    } SetVelocityObj, *SetVelocityStruct;

//stops the chassis completely
typedef struct StopChassis
    {
        unsigned char header = 0xE7;
        unsigned short length = 1;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type = 9;
        bool stop;
        unsigned short footer = 0;
    } StopChassisObj, *StopChassisStruct;

}; //namespace cv
#endif