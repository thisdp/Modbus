#include "Modbus.h"
/* 485 Modbus 协议 */


ModbusRS485::ModbusRS485(HardwareSerial& serial, CRC16 *modbusCRC): RS485(serial), txFrame(modbusCRC), rxFrame(modbusCRC) {
  onReceived = 0;
  timeOut = 0;
  stopDelay = 0;
  failPacks = 0;
  totalPacks = 0;
  clear();
}

void ModbusRS485::clear(){
  state = ModbusRS485::WaitStation;
  failType = ModbusRS485::RcvNoFail; //Clear
  rxFrame.validDataLength = 0;
}

bool ModbusRS485::update(){
  while(available()){
    uint8_t d = read();
    //Serial.println(d);
    switch(state){
      case ModbusRS485::WaitStation:  //Waiting Station
        state = ModbusRS485::WaitFunctionCode; //Wait Function Code
        received = 0; //Reset Length
        rxFrame.buffer[received++] = d;  //Push Station into rxBuffer
      break;
      case ModbusRS485::WaitFunctionCode: //Waiting Function Code
        state = ModbusRS485::WaitData;
        rxFrame.buffer[received++] = d;  //Push Function Code into rxBuffer
      break;
      case ModbusRS485::WaitData:
        rxFrame.buffer[received++] = d;  //Push Data into rxBuffer
      break;
    }
    lastTick = micros();
    if(received >= 384) return 0;
  }
  return 1;
}

void ModbusRS485::setStopDelay(uint32_t argStopDelay){
  stopDelay = argStopDelay;
  if(timeOut <= stopDelay){
    setReceiveTimeOut(stopDelay); //Make sure timeout is larger than stopDelay
  }
}
void ModbusRS485::setReceiveTimeOut(uint32_t argTime){
  timeOut = argTime;
}

void ModbusRS485::printFailType(Stream& stream){
  stream.print(toFailType(failType));
}

const char* ModbusRS485::toFailType(uint8_t failTypeID){
  switch(failTypeID){
    case RcvWaitTimedout:
      return "Timedout";
    case RcvVerifyFailed:
      return "Verify Failed";
    case RcvOverflow:
      return "Overflow";
    case RcvUnsupportedFunctionCode:
      return "Invalid Function Code";
  }
  return "Unknown";
}

void ModbusRS485::clearStatics(){
  failPacks = 0;
  totalPacks = 0;
}

/*Modbus Master*/
ModbusRS485Master::ModbusRS485Master(HardwareSerial& serial, CRC16 *modbusCRC) : ModbusRS485(serial,modbusCRC){}

void ModbusRS485Master::processPack(){
  if(rxFrame.castResponse()){
    if(rxFrame.pack->isDiagnosePack()){
      //todo
    }else{
      verifyRxFrameCRC();
    }
  }else{
    failType = ModbusRS485::RcvUnsupportedFunctionCode;
  }
  totalPacks ++;
  if(failType != RcvNoFail){
    failPacks ++;
  }
}

void ModbusRS485Master::onGetPack(){
  waitSlaveResponse = false;  //结束等待从机返回
  if(onReceived) onReceived(this);
}

void ModbusRS485Master::begin(size_t baud, uint32_t config, int8_t rxPin, int8_t txPin, int8_t dePin, int8_t rePin, bool readBack, uint32_t pWaitSlaveTimedout){
  RS485::begin(baud,config,rxPin,txPin,dePin,rePin,readBack);
  stopDelay = ceil(3.5*1000000.0/baud);
  sendBackDelay = stopDelay*80;
  waitSlavePackTimedout = pWaitSlaveTimedout;
}

void ModbusRS485Master::begin(RS485Config conf,uint32_t pWaitSlaveTimedout){
  RS485::begin(conf);
  stopDelay = ceil(3.5*1000000.0/conf.baudrate);
  sendBackDelay = stopDelay*80;
  waitSlavePackTimedout = pWaitSlaveTimedout;
}

void ModbusRS485Master::update(){
  if(transmitOnUpdateFlag && isSendBackDelayComplete()){
    transmit(transmitTargetStation);
    transmitTargetStation = 0;
    transmitOnUpdateFlag = false;
  }
  uint8_t incoming = available();
  if(state != ModbusRS485::WaitStation && !incoming && isTimedout()){
    if(received >= 4){
      rxFrame.validDataLength = received;
      onGetPack();
    }else{
      failType = ModbusRS485::RcvWaitTimedout;
      onGetPack();
    }
    clear();
  }
  if(waitSlaveResponse){
    if(micros()-waitSlavePackTick > waitSlavePackTimedout){ //超时
      setReceiveWaitTimedout();
      if(onReceived) onReceived(this);
      waitSlaveResponse = false;  //结束等待从机返回
      clear();
    }
  }
  if(!ModbusRS485::update()){ //数据接收过多
    failType = ModbusRS485::RcvOverflow; //溢出
    rxFrame.validDataLength = received;
    onGetPack();
    clear();
  }
}

bool ModbusRS485Master::availableToTransmit(){
  if(waitSlaveResponse && (micros()-waitSlavePackTick <= waitSlavePackTimedout)) //如果是主机正在等待从机回复且没有超时
    return false; //返回不能发送
  return true;
}

void ModbusRS485Master::transmitOnUpdate(uint8_t targetStation){
  transmitTargetStation = targetStation;
  transmitOnUpdateFlag = true;
  sendBackStartTick = micros();
}

bool ModbusRS485Master::transmit(uint8_t targetStation){
  //if(!availableToTransmit()) return false;
  beginTransmission();
  *(txFrame.station) = targetStation; //设置地址
  transmitFrame();
  endTransmission();
  waitSlaveResponse = true;
  waitSlavePackTick = micros();
  return true;
}

bool ModbusRS485Master::transmitRaw(uint8_t targetStation, uint16_t length){
  //if(!availableToTransmit()) return false;
  beginTransmission();
  *(txFrame.station) = targetStation; //设置地址
  transmitFrameRaw(length);
  endTransmission();
  waitSlaveResponse = true;
  waitSlavePackTick = micros();
  return true;
}



/*Modbus Slave*/
ModbusRS485Slave::ModbusRS485Slave(HardwareSerial& serial, CRC16 *modbusCRC) : ModbusRS485(serial, modbusCRC){}

void ModbusRS485Slave::processPack(){
  if(rxFrame.castRequest()){
    verifyRxFrameCRC();
  }else{
    failType = ModbusRS485::RcvUnsupportedFunctionCode;
  }
  totalPacks ++;
  if(failType != RcvNoFail){
    failPacks ++;
  }
}

void ModbusRS485Slave::onGetPack(){
  isAllowedToTransmit = true;  //允许回复数据
  if(onReceived) onReceived(this);
}

void ModbusRS485Slave::begin(uint8_t pStation, size_t baud, uint32_t config, int8_t rxPin, int8_t txPin, int8_t dePin, int8_t rePin, bool readBack){
  RS485::begin(baud,config,rxPin,txPin,dePin,rePin,readBack);
  station = pStation;
  stopDelay = ceil(3.5*1000000.0/baud);
  sendBackDelay = stopDelay*80;
}

void ModbusRS485Slave::begin(uint8_t pStation,RS485Config conf){
  RS485::begin(conf);
  station = pStation;
  stopDelay = ceil(3.5*1000000.0/conf.baudrate);
  sendBackDelay = stopDelay*80;
}

void ModbusRS485Slave::update(){
  if(transmitOnUpdateFlag && isSendBackDelayComplete()){
    //Serial.println("Send on update");
    if(availableToTransmit()){
      transmit();
    }
    transmitOnUpdateFlag = false;
  }
  uint8_t incoming = available();
  if(state != ModbusRS485::WaitStation && !incoming && isTimedout()){
    /*Serial.println("------");
    Serial.println(micros());
    Serial.println(lastTick);
    Serial.println(micros()-lastTick);
    Serial.println(stopDelay);
    Serial.println("------");*/
    if(received >= 4){
      rxFrame.validDataLength = received;
      onGetPack();
    }else{
      failType = ModbusRS485::RcvWaitTimedout;
      onGetPack();
    }
    clear();
  }
  if(!ModbusRS485::update()){ //数据接收过多
    failType = ModbusRS485::RcvOverflow; //溢出
    rxFrame.validDataLength = received;
    onGetPack();
    clear();
  }
}

bool ModbusRS485Slave::availableToTransmit(){
  return isAllowedToTransmit;
}

void ModbusRS485Slave::transmitOnUpdate(){
  transmitOnUpdateFlag = true;
}

bool ModbusRS485Slave::transmit(){
  //if(!availableToTransmit()) return false;
  beginTransmission();
  *(txFrame.station) = station; //设置地址
  transmitFrame();
  endTransmission();
  isAllowedToTransmit = false;
  return true;
}

bool ModbusRS485Slave::transmitRaw(uint16_t length){
  //if(!availableToTransmit()) return false;
  beginTransmission();
  transmitFrameRaw(length);
  endTransmission();
  isAllowedToTransmit = false;
  return true;
}

uint8_t ModbusRS485Slave::getStation(){
  return station;
}

bool ModbusRS485Slave::setStation(uint8_t newStation){
  if(newStation == 0) return false; //Station ID 0 is invalid
  if(newStation > 247) return false; //Station Range is invalid
  station = newStation;
  return true;
}
