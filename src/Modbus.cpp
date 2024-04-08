#include "Modbus.h"
/* 485 Modbus 协议 */

ModbusRS485::ModbusRS485(int uart_nr): RS485(uart_nr){
  onReceived = 0;
  clear();
}

ModbusRS485::ModbusRS485(const HardwareSerial& serial): RS485(serial){
  onReceived = 0;
  clear();
}

void ModbusRS485::clear(){
  state = ModbusRS485::WaitStation;
  failType = ModbusRS485::RcvNoFail; //Clear
}

void ModbusRS485::update(){
  while(available()){
    lastTick = micros();
    uint8_t d = read();
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
  }
}

/*Modbus Master*/
ModbusRS485Master::ModbusRS485Master(int uart_nr): ModbusRS485(uart_nr){}

ModbusRS485Master::ModbusRS485Master(const HardwareSerial& serial): ModbusRS485(serial){}

void ModbusRS485Master::onGetPack(){
  waitSlaveResponse = false;  //结束等待从机返回
  rxFrame.castResponse();
  verifyRxFrameCRC();
  if(onReceived) onReceived(this);
}

void ModbusRS485Master::begin(size_t baud, uint32_t config, int8_t rxPin, int8_t txPin, int8_t dePin, int8_t rePin, bool readBack, uint32_t pWaitSlaveTimedout){
  RS485::begin(baud,config,rxPin,txPin,dePin,rePin,readBack);
  stopDelay = ceil(3.5*80*1000000.0/baud);
  waitSlavePackTimedout = pWaitSlaveTimedout;
}

void ModbusRS485Master::begin(RS485Config conf,uint32_t pWaitSlaveTimedout){
  RS485::begin(conf);
  stopDelay = ceil(3.5*80*1000000.0/conf.baudrate);
  waitSlavePackTimedout = pWaitSlaveTimedout;
}

void ModbusRS485Master::update(){
  uint8_t incoming = available();
  if(state != ModbusRS485::WaitStation && !incoming && isTimedout(stopDelay)){
    if(received >= 4) onGetPack();
    clear();
  }
  if(waitSlaveResponse){
    if(millis()-waitSlavePackTick > waitSlavePackTimedout){ //超时
      setReceiveWaitTimedout();
      if(onReceived) onReceived(this);
      waitSlaveResponse = false;  //结束等待从机返回
      clear();
    }
  }
  ModbusRS485::update();
}

bool ModbusRS485Master::availableToTransmit(){
  if(waitSlaveResponse && (millis()-waitSlavePackTick <= waitSlavePackTimedout)) //如果是主机正在等待从机回复且没有超时
    return false; //返回不能发送
  return true;
}

bool ModbusRS485Master::transmit(uint8_t targetStation){
  if(!availableToTransmit()) return false;
  beginTransmission();
  *(txFrame.station) = targetStation; //设置地址
  transmitFrame();
  endTransmission();
  waitSlaveResponse = true;
  waitSlavePackTick = millis();
  return true;
}

/*Modbus Slave*/

ModbusRS485Slave::ModbusRS485Slave(int uart_nr): ModbusRS485(uart_nr){}

ModbusRS485Slave::ModbusRS485Slave(const HardwareSerial& serial): ModbusRS485(serial){}

void ModbusRS485Slave::onGetPack(){
  isAllowedToTransmit = true;  //允许回复数据
  rxFrame.castRequest();
  verifyRxFrameCRC();
  if(onReceived) onReceived(this);
}

void ModbusRS485Slave::begin(uint8_t pStation, size_t baud, uint32_t config, int8_t rxPin, int8_t txPin, int8_t dePin, int8_t rePin, bool readBack){
  RS485::begin(baud,config,rxPin,txPin,dePin,rePin,readBack);
  station = pStation;
  stopDelay = ceil(3.5*80*1000000.0/baud);
}

void ModbusRS485Slave::begin(uint8_t pStation,RS485Config conf){
  RS485::begin(conf);
  station = pStation;
  stopDelay = ceil(3.5*80*1000000.0/conf.baudrate);
}

void ModbusRS485Slave::update(){
  uint8_t incoming = available();
  if(state != ModbusRS485::WaitStation && !incoming && isTimedout(stopDelay)){
    if(received >= 4) onGetPack();
    clear();
  }
  ModbusRS485::update();
}

bool ModbusRS485Slave::availableToTransmit(){
  return isAllowedToTransmit;
}

bool ModbusRS485Slave::transmit(){
  if(!availableToTransmit()) return false;
  beginTransmission();
  *(txFrame.station) = station; //设置地址
  transmitFrame();
  endTransmission();
  isAllowedToTransmit = false;
  return true;
}