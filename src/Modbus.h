#pragma once
#include "HardwareSerial.h"
#include "CRC16.h"
#include "RS485.h"

#define ModbusStopWord 3.5

#define ModbusMaster 0 //Station = 0 为主机
#define ModbusSlave(x) x

extern CRC16 ModbusCRC;

#define ModbusRcvFailType_WaitPackTimedout 0x00
#define ModbusRcvFailType_PackVerifyFailed 0x02

typedef void(*ModbusCallbackOnReceived)(uint8_t station, uint8_t functionCode, uint8_t *data, uint8_t dataLength);  //dataLength不包括station functionCode和CRC
typedef void(*ModbusCallbackOnReceiveFailed)(uint8_t failType);
class ModbusReceiver{
public:
  constexpr static uint8_t WaitStation = 0x00;
  constexpr static uint8_t WaitFunctionCode = 0x01;
  constexpr static uint8_t WaitData = 0x02;

  uint32_t lastTick;
  uint8_t received;
  uint8_t state;
  uint8_t buffer[256];
  bool isTimedout();
  inline bool isPackReadTimedout(){
    if(receiveState == ModbusRcvState_WaitStation) return false;
    if(micros()-lastReceiveTick > stopDelay) return true;
    return false;
  }
};

class ModbusMaster : private RS485 {
public:
	ModbusMaster(int uart_nr);
  ModbusMaster(const HardwareSerial& serial);
  void begin(size_t baud, uint8_t station, uint32_t config = SERIAL_8N1, int8_t rxPin=-1, int8_t txPin=-1, int8_t dePin=-1, int8_t rePin = -1, bool readBack = false, uint32_t pWaitSlaveTimedout = 10);
  void update();
  void clear();
  void complete();
  bool send(uint8_t targetStation, uint8_t functionCode, uint8_t *data, uint8_t size);
  bool availableToSend();
  ModbusCallbackOnReceived onReceived;
  ModbusCallbackOnReceiveFailed onReceiveFailed;
  ModbusReceiver receiver;
  //从机部分
  uint8_t slaveIsAllowedToSend;
  //主机部分
  uint8_t masterWaitSlaveResponse;
  uint32_t masterWaitSlavePackTick;
  uint32_t masterWaitSlavePackTimedout;
private:
};

class ModbusSlave : private RS485 {
public:
	ModbusSlave(int uart_nr);
  ModbusSlave(const HardwareSerial& serial);
  void begin(size_t baud, uint8_t station, uint32_t config = SERIAL_8N1, int8_t rxPin=-1, int8_t txPin=-1, int8_t dePin=-1, int8_t rePin = -1, bool readBack = false, uint32_t pWaitSlaveTimedout = 10);
  void update();
  void clear();
  void complete();
  bool send(uint8_t targetStation, uint8_t functionCode, uint8_t *data, uint8_t size);
  bool availableToSend();
  uint8_t station;
  ModbusCallbackOnReceived onReceived;
  ModbusCallbackOnReceiveFailed onReceiveFailed;
  uint32_t lastReceiveTick;
  uint8_t receivedLength;
  uint8_t receiveState;
  uint8_t receiveBuffer[256];
  //从机部分
  uint8_t slaveIsAllowedToSend;
  //主机部分
  uint8_t masterWaitSlaveResponse;
  uint32_t masterWaitSlavePackTick;
  uint32_t masterWaitSlavePackTimedout;
private:
  bool isPackReadTimedout();
};

#pragma pack(push, 1) //1字节对齐

class ModbusFrame{
public:
  uint8_t station;
};

class ModbusBasePack{
public:
  uint8_t functionCode;
};

/****************读线圈寄存器****************/
//请求
class MBReadCoilRegisterRequest : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x01;
  uint8_t startAddressHigh;
  uint8_t startAddressLow;
  uint8_t quantityHigh;
  uint8_t quantityLow;
  inline uint16_t getStartAddress(){
    return (startAddressHigh<<8)|startAddressLow;
  }
  inline uint16_t getQuantity(){
    return (quantityHigh<<8)|quantityLow;
  }
  inline void write(Stream &s){
    s.write((uint8_t*)this,sizeof(MBReadCoilRegisterRequest));
  }
};
//回复
class MBReadCoilRegisterResponse : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x01;
  uint8_t bytes;
  uint8_t *outputs;
  inline void processRequest(MBReadCoilRegisterRequest &req){

  }
  inline void write(Stream &s){
    s.write((uint8_t*)this,sizeof(MBReadCoilRegisterResponse)-sizeof(size_t));
    s.write(outputs,bytes);
  }
};

/****************读离散输入寄存器****************/
//请求 (和 读线圈寄存器 基本一致)
class MBReadDiscreteInputRegisterRequest : public MBReadCoilRegisterRequest{
public:
  static const uint8_t FunctionCode = 0x02;
  inline void write(Stream &s){
    s.write((uint8_t*)this,sizeof(MBReadDiscreteInputRegisterRequest));
  }
};
//回复 (和 读线圈寄存器 基本一致)
class MBReadDiscreteInputRegisterResponse : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x02;
  uint8_t bytes;
  uint8_t *outputs;
  inline void processRequest(MBReadDiscreteInputRegisterRequest &req){

  }
  inline void write(Stream &s){
    s.write((uint8_t*)this,sizeof(MBReadDiscreteInputRegisterResponse)-sizeof(size_t));
    s.write(outputs,bytes);
  }
};

/****************读保持寄存器****************/
//请求 (和 读线圈寄存器 基本一致)
class MBReadHoldingRegisterRequest : public MBReadCoilRegisterRequest{
public:
  static const uint8_t FunctionCode = 0x03;
  inline void write(Stream &s){
    s.write((uint8_t*)this,sizeof(MBReadHoldingRegisterRequest));
  }
};
//回复
class MBReadHoldingRegisterResponse : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x03;
  uint8_t bytes;
  uint8_t *outputs;
  inline void processRequest(MBReadHoldingRegisterRequest &req){

  }
  inline void write(Stream &s){
    s.write((uint8_t*)this,sizeof(MBReadHoldingRegisterResponse)-sizeof(size_t));
    s.write(outputs,bytes);
  }
};

/****************读输入寄存器****************/
//请求 (和 读保持寄存器 基本一致)
class MBReadInputRegisterRequest : public MBReadHoldingRegisterRequest{
public:
  static const uint8_t FunctionCode = 0x04;
  inline void write(Stream &s){
    s.write((uint8_t*)this,sizeof(MBReadInputRegisterRequest));
  }
};
//回复
class MBReadInputRegisterResponse : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x04;
  uint8_t bytes;
  uint8_t *outputs;
  inline void processRequest(MBReadInputRegisterRequest &req){

  }
  inline void write(Stream &s){
    s.write((uint8_t*)this,sizeof(MBReadInputRegisterResponse)-sizeof(size_t));
    s.write(outputs,bytes);
  }
};

/****************写单个线圈寄存器****************/
//请求
class MBWriteCoilRegisterRequest : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x05;
  uint8_t addressHigh;
  uint8_t addressLow;
  uint8_t valueHigh;
  uint8_t valueLow;
  uint16_t getAddress(){
    return (addressHigh<<8)|addressLow;
  }
  uint16_t getValue(){
    return (valueHigh<<8)|valueLow;
  }
  inline void write(Stream &s){
    s.write((uint8_t*)this,sizeof(MBWriteCoilRegisterRequest));
  }
};
//回复 ( 与请求报文一样 )
class MBWriteCoilRegisterResponse : public MBWriteCoilRegisterRequest{
public:
};

/****************写单个保持寄存器****************/
//请求
class MBWriteHoldingRegisterRequest : public MBWriteCoilRegisterRequest{
public:
  static const uint8_t FunctionCode = 0x06;
  inline void write(Stream &s){
    s.write((uint8_t*)this,sizeof(MBWriteHoldingRegisterRequest));
  }
};
//回复 ( 与请求报文一样 )
class MBWriteHoldingRegisterResponse : public MBWriteHoldingRegisterRequest{
public:
};

/****************写多个线圈寄存器****************/
//请求
class MBWriteMultipleCoilRegistersRequest : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x0F;
  uint8_t startAddressHigh;
  uint8_t startAddressLow;
  uint8_t quantityHigh;
  uint8_t quantityLow;
  uint8_t bytes;
  uint8_t *outputs;
  inline uint16_t getStartAddress(){
    return (startAddressHigh<<8)|startAddressLow;
  }
  inline uint16_t getQuantity(){
    return (quantityHigh<<8)|quantityLow;
  }
  inline uint8_t getBytes(){
    return bytes;
  }
  inline void write(Stream &s){
    s.write((uint8_t*)this,sizeof(MBWriteMultipleCoilRegistersRequest)-sizeof(size_t));
    s.write(outputs,bytes);
  }
};
//回复
class MBWriteMultipleCoilRegistersResponse : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x0F;
  uint8_t startAddressHigh;
  uint8_t startAddressLow;
  uint8_t quantityHigh;
  uint8_t quantityLow;
  inline void write(Stream &s){
    s.write((uint8_t*)this,sizeof(MBWriteMultipleCoilRegistersResponse));
  }
};

/****************写多个保持寄存器****************/
//请求
class MBWriteMultipleHoldingRegistersRequest : public MBWriteMultipleCoilRegistersRequest{
public:
  static const uint8_t FunctionCode = 0x10;
};
//回复
class MBWriteMultipleHoldingRegistersResponse : public MBWriteMultipleCoilRegistersResponse{
public:
  static const uint8_t FunctionCode = 0x10;
};

#pragma pack(pop)