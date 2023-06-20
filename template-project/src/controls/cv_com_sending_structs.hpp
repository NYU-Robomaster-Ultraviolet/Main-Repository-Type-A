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

};// namespace CV
#endif