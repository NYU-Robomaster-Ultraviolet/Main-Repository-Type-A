#ifndef CV_COMMAND_HPP_
#define CV_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "drivers.hpp"
#include "subsystems/gimbal/gimbal_subsystem.hpp"
namespace gimbal{
class CvCommand : public tap::control::Command {
public:
    CvCommand(GimbalSubsystem *const gimbal, src::Drivers *drivers);

    CvCommand(const CvCommand &other) = delete;

    CvCommand &operator=(const CvCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "cv command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    float findRotation(const float& destination) const;

    static int writeToUart(src::Drivers *drivers, std::string s);

    static int writeToUart(src::Drivers *drivers, char *s, int n);

    static int readFromUart(src::Drivers *drivers, char *buffer);

    void UnPackMsgs(src::Drivers *drivers, char *buffer);

    void sendAutoAimMsg(src::Drivers *drivers, int pitch, int yaw, int hasTarget);

    void updateGimbalError(int pitch, int yaw);


private:
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

typedef struct header {
    unsigned char header;
    unsigned short length;
    unsigned char empty1;
    unsigned char empty2;
    unsigned short msg_type;
} *Header;
    GimbalSubsystem* gimbal;
    src::Drivers* drivers;
}; //CvCommand


}// namespace cv

#endif