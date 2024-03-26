#include "Modbus.h"
/* 485 Modbus 协议 */

CRC16 ModbusCRC(CRC16MODBUS);
uint8_t ModbusSizeMapper[128] = {0};

/*Modbus收发器*/
ModbusRS485::ModbusRS485(int uart_nr): RS485(uart_nr){
  onReceived = 0;
  onReceiveFailed = 0;
  station = 0;
  clear();
}

ModbusRS485::ModbusRS485(const HardwareSerial& serial): RS485(serial){
  onReceived = 0;
  onReceiveFailed = 0;
  station = 0;
  clear();
}

void ModbusRS485::clear(){
  receiveState = ModbusRcvState_WaitStation;
}

bool ModbusRS485::isMaster(){
  return station == 0;
}


void ModbusRS485::begin(size_t baud, uint8_t pStation, uint32_t config, int8_t rxPin, int8_t txPin, int8_t dePin, int8_t rePin, bool readBack, uint32_t pWaitSlaveTimedout){
  RS485::begin(baud,config,rxPin,txPin,dePin,rePin,readBack);
  stopDelay = ceil(ModbusStopWord*10000000.0/baud);
  station = pStation;
  masterWaitSlavePackTimedout = pWaitSlaveTimedout;
}

void ModbusRS485::complete(){ //如果完成接收数据包
  if(receivedLength < 4) return;
  if(!onReceived) return;
  uint16_t CRC = *((uint16_t*)(&receiveBuffer[receivedLength-2]));
  ModbusCRC.clear();
  ModbusCRC.update((uint8_t*)receiveBuffer,receivedLength-2);
  uint16_t aCRC = ModbusCRC.get();
  if(ModbusCRC.get() == CRC){
    if(!isMaster()){ //如果是从机
      slaveIsAllowedToSend = true;  //允许回复数据
    }else{
      masterWaitSlaveResponse = false;  //结束等待从机返回
    }
    onReceived(receiveBuffer[0],receiveBuffer[1],receiveBuffer+2,receivedLength-4);  //Station, FunctionCode
  }else{
    if(onReceiveFailed) onReceiveFailed(ModbusRcvFailType_PackVerifyFailed);
  }
}

void ModbusRS485::update(){
  uint8_t incoming = available();
  if(receiveState != ModbusRcvState_WaitStation && !incoming && isPackReadTimedout()){
    complete();
    clear();
  }
  if(isMaster()){
    if(masterWaitSlaveResponse){
      if(millis()-masterWaitSlavePackTick > masterWaitSlavePackTimedout){ //超时
        if(onReceiveFailed) onReceiveFailed(ModbusRcvFailType_WaitPackTimedout);
        masterWaitSlaveResponse = false;  //结束等待从机返回
      }
    }
  }
  while(available()){
    lastReceiveTick = micros();
    uint8_t d = read();
    switch(receiveState){
      case ModbusRcvState_WaitStation:  //Waiting Station
        receiveState = ModbusRcvState_WaitFunctionCode; //Wait Function Code
        receivedLength = 0; //Reset Length
        receiveBuffer[receivedLength++] = d;  //Push Station into receiveBuffer
      break;
      case ModbusRcvState_WaitFunctionCode: //Waiting Function Code
        receiveState = ModbusRcvState_WaitData;
        receiveBuffer[receivedLength++] = d;  //Push Function Code into receiveBuffer
      break;
      case ModbusRcvState_WaitData:
        receiveBuffer[receivedLength++] = d;  //Push Data into receiveBuffer
      break;
    }
  }
}

bool ModbusRS485::availableToSend(){
  if(isMaster()){
    if(masterWaitSlaveResponse && (millis()-masterWaitSlavePackTick <= masterWaitSlavePackTimedout)) //如果是主机正在等待从机回复且没有超时
    return false; //返回不能发送
  } else {
    return slaveIsAllowedToSend;
  }
  return true;
}

bool ModbusRS485::send(uint8_t targetStation, uint8_t functionCode, uint8_t *data, uint8_t dataLength){
  if(!availableToSend()) return false;
  if(!isMaster()) targetStation = station;  //只有主站才能指定地址
  ModbusCRC.clear();
  ModbusCRC.update(targetStation);
  ModbusCRC.update(functionCode);
  ModbusCRC.update(data,dataLength);
  beginTransmission();
  write(station);
  write(functionCode);
  write(data,dataLength);
  uint16_t crc = ModbusCRC.get();
  write((uint8_t*)&crc,2);
  endTransmission();
  if(isMaster()){ //如果是主机，则需要等待从机回复
    masterWaitSlaveResponse = true;
    masterWaitSlavePackTick = millis();
  }else{  //如果是从机，则回复完之后不允许发送
    slaveIsAllowedToSend = false;
  }
  return true;
}
