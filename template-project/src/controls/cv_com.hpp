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

    float getYaw() const {return yaw;}
    float getPitch() const {return pitch;}

    void invalidateAngle() {validAngle = false;}

    bool validReading() const {return validAngle;}

    bool online() const {return !timeout.isExpired();}

    void init();

    typedef struct autoAimStruct {
    unsigned char header;
    unsigned short length;
    unsigned char empty1;
    unsigned char empty2;
    unsigned short msg_type;
    float pitch;
    float yaw;
    unsigned char hasTarget;
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

typedef struct floating {
    float pitch;
} floatingObj, *floatingStruct;
private:
    src::Drivers* drivers;

    //paw then pitch
    float yaw;
    float pitch;
    bool validAngle = false;
    tap::arch::MilliTimeout timeout;
}; // class CVCom
#endif