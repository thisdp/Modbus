#pragma once
#include "HardwareSerial.h"
#include "RS485.h"
#include "ModbusPack.h"

class ModbusRS485;
typedef void(*ModbusCallbackOnReceived)(ModbusRS485 *modbusController);

class ModbusRS485 : public RS485{
public:
  constexpr static uint8_t WaitStation = 0x00;
  constexpr static uint8_t WaitFunctionCode = 0x01;
  constexpr static uint8_t WaitData = 0x02;

  constexpr static uint8_t RcvNoFail = 0x00;
  constexpr static uint8_t RcvWaitTimedout = 0x01;
  constexpr static uint8_t RcvVerifyFailed = 0x02;
  constexpr static uint8_t RcvOverflow = 0x03;
  constexpr static uint8_t RcvUnsupportedFunctionCode = 0x04;
  
  ModbusCallbackOnReceived onReceived;
  uint32_t stopDelay;
  uint32_t timeOut;
  uint32_t sendBackStartTick;
  uint32_t sendBackDelay;

  uint32_t lastTick;
  uint32_t failPacks;
  uint32_t totalPacks;
  uint16_t received;
  uint8_t state;
  uint8_t failType;

  ModbusFrame txFrame;
  ModbusFrame rxFrame;

	ModbusRS485(int uart_nr, CRC16 *modbusCRC = 0);
  ModbusRS485(const HardwareSerial& serial, CRC16 *modbusCRC = 0);
  bool update();
  void clear();
  void setStopDelay(uint32_t argStopDelay);
  void setReceiveTimeOut(uint32_t argTime);
  void printFailType(Stream& stream);
  void clearStatics();
  inline bool isStationValid(uint8_t st){ return st >= 1 && st <= 247; }
protected:
  inline void transmitFrame(){
    applyTxFrameCRC();
    txFrame.write(*this);
  }

  inline void transmitFrameRaw(uint16_t length){
    txFrame.writeRaw(*this,length);
  }
  
  inline void verifyRxFrameCRC(){
    if(!rxFrame.verifyCRC()){
      failType = RcvVerifyFailed;
    }
  }
  inline void applyTxFrameCRC(){
    txFrame.applyCRC();
  }
  inline uint8_t getFailType(){
    return failType;
  }
  inline bool isTimedout(){
    if(state == WaitStation) return false;
    if(micros()-lastTick > timeOut) return true;
    return false;
  }
  inline bool isSendBackDelayComplete(){
    if(micros()-sendBackStartTick > sendBackDelay)
      return true;
    else
      return false;
  }
  inline void setReceiveWaitTimedout(){
    failType = RcvWaitTimedout;
  }
};

class ModbusRS485Master : public ModbusRS485 {
public:
	ModbusRS485Master(int uart_nr, CRC16 *modbusCRC = 0);
  ModbusRS485Master(const HardwareSerial& serial, CRC16 *modbusCRC = 0);
  void begin(size_t baud, uint32_t config = SERIAL_8N1, int8_t rxPin=-1, int8_t txPin=-1, int8_t dePin=-1, int8_t rePin = -1, bool readBack = false, uint32_t pWaitSlaveTimedout = 500*1000);
  void begin(RS485Config conf,uint32_t pWaitSlaveTimedout = 500*1000);
  void update();
  bool availableToTransmit();
  void transmitOnUpdate(uint8_t targetStation);
  bool transmit(uint8_t targetStation);
  bool transmitRaw(uint8_t targetStation, uint16_t length);
  void processPack();
private:
  void onGetPack();
  bool transmitOnUpdateFlag;
  uint8_t transmitTargetStation;
  uint32_t waitSlavePackTick;
  uint32_t waitSlavePackTimedout;
  uint8_t waitSlaveResponse;
};

class ModbusRS485Slave : public ModbusRS485 {
public:
	ModbusRS485Slave(int uart_nr, CRC16 *modbusCRC = 0);
  ModbusRS485Slave(const HardwareSerial& serial, CRC16 *modbusCRC = 0);
  void begin(uint8_t station, size_t baud, uint32_t config = SERIAL_8N1, int8_t rxPin=-1, int8_t txPin=-1, int8_t dePin=-1, int8_t rePin = -1, bool readBack = false);
  void begin(uint8_t station, RS485Config conf);
  void update();
  bool availableToTransmit();
  void transmitOnUpdate();
  bool transmit();
  bool transmitRaw(uint16_t length);
  void processPack();
  uint8_t getStation();
  void setStation(uint8_t station);
private:
  void onGetPack();
  bool transmitOnUpdateFlag;
  uint8_t station;
  uint8_t isAllowedToTransmit;
};