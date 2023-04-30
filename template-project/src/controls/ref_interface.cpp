#include "ref_interface.hpp"
#include "drivers.hpp"

void RefInterface::updateData(){
   tap::communication::serial::RefSerialData::Rx::RobotData data = drivers->refSerial.getRobotData();
   currHP = data.currentHp;
   //robotID = data.robotId;
}