#ifndef CV_COM_HPP_
#define CV_COM_HPP_
#include "modm/math/geometry/angle.hpp"
#include "tap/architecture/timeout.hpp"


namespace src{
    class Drivers;
}

class CVCom{
public:
    CVCom(src::Drivers* drivers);

    int writeToUart(const std::string& s);

    int writeToUart(char *s, int n);

    int readFromUart(char *buffer);

    void UnPackMsgs(char *buffer);

    void sendAutoAimMsg(int pitch, int yaw, int hasTarget);

    void update();

    const std::pair<float, float> getAngle() const {return angle;}

    void invalidateAngle() {validAngle = false;}

    bool validReading() const {return validAngle;}

    typedef struct autoAimStruct {
    unsigned char header;
    unsigned short length;
    unsigned char empty1;
    unsigned char empty2;
    unsigned short msg_type;
    unsigned short pitch;
    unsigned short yaw;
    bool hasTarget;
    unsigned short footer;
} AutoAimStructObj, *AutoAimStruct;

typedef struct enableStruct {
    unsigned char header;
    unsigned short length;
    unsigned char empty1;
    unsigned char empty2;
    unsigned short msg_type;
    bool start;
    unsigned short footer;
} EnableStructObj, *EnableStruct;

typedef struct header {
    unsigned char header;
    unsigned short length;
    unsigned char empty1;
    unsigned char empty2;
    unsigned short msg_type;
} *Header;
private:
    src::Drivers* drivers;

    //paw then pitch
    std::pair<float, float> angle;
    bool validAngle;
}; // class CVCom
#endif