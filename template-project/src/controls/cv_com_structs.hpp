#ifndef CV_COM_STRUCTS_HPP_
#define CV_COM_STRUCTS_HPP_
namespace cv{
//receive message about movement and targeting
typedef struct autoAimStruct
    {
        unsigned char header = 0xE7;
        unsigned short length = 8;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type = 5;
        int pitch;
        int yaw;
        unsigned char hasTarget;
        unsigned short footer = 0;
    } AutoAimStructObj, *AutoAimStruct;

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

// message_type: 1
typedef struct alignRequestStruct
    {
        unsigned char header = 0xE7;
        unsigned short length;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type = 1;
        unsigned short x;
        unsigned short y;
        unsigned short r;
        unsigned short footer = 0;
    } AlignRequestStructObj, *AlignRequestStruct;

// message_type: 3
typedef struct alignFinishStruct
    {
        unsigned char header = 0xE7;
        unsigned short length;
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
        int gimbalX;  // gimbal yaw movement
        int gimbalY;  // gimbal pitch movement
        // 0 : default movement, 1 : beyblade, 2 : beyblade backwards
        unsigned char mode;  // used to switch command behavior
        unsigned short footer = 0;
    } GimbalMoveStructObj, *GimbalMoveStruct;

    // Receive
typedef struct enableStruct
    {
        unsigned char header = 0xE7;
        unsigned short length;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type;
        bool start;
        unsigned short footer = 0;
    } EnableStructObj, *EnableStruct;

// Send/recieve
typedef struct header
    {
        // unsigned char
        unsigned char header = 0xE7;
        // unsigned short
        unsigned short length;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type;
    } *Header;

//chassis movement structs
// message_type: 2
typedef struct chassisMoveStruct
    {
        unsigned char header = 0xE7;
        unsigned short length;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type = 2;  // 2
        // value should be between -1000 and 1000;
        int chassisX;  // chassis x axis movement
        int chassisY;  // chassis y axis movement
        int chassisR;  // rotational movement
        // 0 : default movement
        unsigned char mode;  // used to switch command behavior
        unsigned short footer = 0;
    } ChassisMoveStructObj, *ChassisMoveStruct;

//straight foward movement
typedef struct MoveStraight
    {
        unsigned char header = 0xE7;
        unsigned short length = 8;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type;
        int distance; //in mm
        int velocity; //m/s, divide by 1000 to get
        unsigned short footer = 0;
    } MoveStraightObj, *MoveStraightStruct;

//straight foward movement
typedef struct SpinChassis
    {
        unsigned char header = 0xE7;
        unsigned short length = 8;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type;
        int angle; //in mm
        int velocity; //degree/s, divide by 1000 to get
        unsigned short footer = 0;
    } SpinChassisObj, *SpinChassisStruct;

//seting power given
typedef struct SetChassisPower
    {
        unsigned char header = 0xE7;
        unsigned short length = 16;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type;
        int xLinearPower;
        int yLinearPower;
        int xAnglePower;
        int yAnglePower;
        unsigned short footer = 0;
    } SetChassisPowerObj, *SetChassisPowerStruct;

typedef struct SetChassisVelocity
    {
        unsigned char header = 0xE7;
        unsigned short length = 16;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type;
        int xVelocity;
        int yVelocity;
        int yawVelocity;
        int pitchVelocity;
        unsigned short footer = 0;
    } SetChassisVelocityObj, *SetChassisVelocityStruct;

typedef struct StopChassis
    {
        unsigned char header = 0xE7;
        unsigned short length = 1;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type;
        bool stop;
        unsigned short footer = 0;
    } StopChassisObj, *StopChassisStruct;

};// namespace CV
#endif