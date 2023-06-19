#include "ref_interface.hpp"
#include "drivers.hpp"

void RefInterface::updateData(){
   tap::communication::serial::RefSerialData::Rx::RobotData data = drivers->refSerial.getRobotData();
   tap::communication::serial::RefSerialData::Rx::GameData gameData = drivers->refSerial.getGameData();
   //general robot data
   returnData.robotID = static_cast<unsigned char>(data.robotId);
   returnData.currHP = data.currentHp;
   returnData.maxHP = data.maxHp;
   returnData.robotLevel = data.robotLevel;

   //match timing and stage
   returnData.stage = static_cast<uint8_t> (gameData.gameStage);
   returnData.timeLeft = gameData.stageTimeRemaining; 

   //All robots HP
   //redHP
   returnData.redHero1 = data.allRobotHp.red.hero1;
   returnData.redEngineer2 = data.allRobotHp.red.engineer2;
   returnData.redStandard3 = data.allRobotHp.red.standard3;
   returnData.redStandard4 = data.allRobotHp.red.standard4;
   returnData.redStandard5 = data.allRobotHp.red.standard5;
   returnData.redSentry7 = data.allRobotHp.red.sentry7;
   returnData.redOutpost = data.allRobotHp.red.outpost;
   returnData.redBase = data.allRobotHp.red.base;
   //blueHP
   returnData.blueHero1 = data.allRobotHp.blue.hero1;
   returnData.blueEngineer2 = data.allRobotHp.blue.engineer2;
   returnData.blueStandard3 = data.allRobotHp.blue.standard3;
   returnData.blueStandard4 = data.allRobotHp.blue.standard4;
   returnData.blueStandard5 = data.allRobotHp.blue.standard5;
   returnData.blueSentry7 = data.allRobotHp.blue.sentry7;
   returnData.blueOutpost = data.allRobotHp.blue.outpost;
   returnData.blueBase = data.allRobotHp.blue.base;

   //returnData.remainingCoins = data.remainingCoins;

   //RFID info (all bools)
   returnData.rfid = static_cast<u_int8_t>(data.rfidStatus.value);

   //chassis data
   returnData.powerUsage = data.chassis.power;
   returnData.powerLimit = data.chassis.powerConsumptionLimit;

   //shooting data
   returnData.heat17ID1 = data.turret.heat17ID1;
   returnData.heat17ID2 = data.turret.heat17ID2;
   returnData.heat42 = data.turret.heat42;

   returnData.heatLimit17ID1 = data.turret.heatLimit17ID1;
   returnData.heatLimit17ID2 = data.turret.heatLimit17ID2;
   returnData.heatLimit42 = data.turret.heatLimit42;

   //armor
   returnData.damagedArmorId = static_cast<u_int8_t> (data.damagedArmorId);
   returnData.damageType = static_cast<u_int8_t> (data.damageType);
   returnData.receivedDpms = data.receivedDps * 1000; //converts dps to dpms to avoid floats

   returnData.warningLevel = data.refereeWarningData.level; //0: none 1: yellow card 2: red card 3: forfeture 
   returnData.warningRobotID = static_cast<u_int8_t>(data.refereeWarningData.foulRobotID);   //Robot ID 1-9 for red, 101-109 for blue
   returnData.lastReceivedWarningRobotTime = data.refereeWarningData.lastReceivedWarningRobotTime;  ///< Last time (in milliseconds) that a warning was received.
}