#pragma once
#include <stdint.h>
#include "CRC16.h"

extern CRC16 ModbusCRC;

#pragma pack(push, 1) //1字节对齐

/*******************************************基础数据类型*******************************************/
class uint16_modbus{
public:
    uint8_t High;
    uint8_t Low;
    uint16_modbus(uint16_t value = 0);
    inline uint16_t get(){
        return (High<<8)|Low;
    }
    inline void set(uint16_t value){
        High = value >> 8;
        Low = value & 0xFF;
    }
};
/*******************************************基类兼数据包工厂*******************************************/
class ModbusBasePack{
public:
  uint8_t *functionCode;
  uint8_t *endOfPack;
  virtual void write(Stream &s);
  virtual uint8_t *cast(uint8_t *buf, bool isNew = false);

  inline uint8_t getStation(){ return *(functionCode-1); }
  inline uint8_t getFunctionCode() { return *functionCode; }
  inline void setEOP(uint8_t *p){ endOfPack = p; };
  inline uint16_t getSize(){ return endOfPack - (functionCode - 1); }
  //virtual void process();
  //ModbusBasePack *CastModbusRequestPack(uint8_t *p);
  //ModbusBasePack *CastModbusResponsePack(uint8_t *p);
  virtual void pushRegisters(bool toTail, uint16_t quant, uint8_t *data) {}
  virtual void popRegisters(bool toTail, uint16_t quant) {}
public: //静态
  static ModbusBasePack* CreateModbusDiagnosePack();
  static ModbusBasePack* CreateModbusRequestPack(uint8_t functionCode);
  static ModbusBasePack* CreateModbusResponsePack(uint8_t functionCode);
};

/*******************************************Modbus帧*******************************************/
class ModbusFrame {
public:
    ModbusFrame();
    ~ModbusFrame();
    uint8_t* station;
    ModbusBasePack* pack;
    uint16_t *crc;
    uint8_t buffer[384];
    uint16_t validDataLength;
    uint8_t* castDiagnose(bool isNew = false);
    uint8_t* castRequest(bool isNew = false);
    uint8_t* castResponse(bool isNew = false);
    uint8_t* createDiagnose(uint8_t functionCode);
    uint8_t* createRequest(uint8_t functionCode);
    uint8_t* createResponse(uint8_t functionCode);
    void copy(ModbusFrame &frame, uint16_t length);
    bool verifyCRC();
    void applyCRC();
    void write(Stream &s);
    void writeRaw(Stream &s, uint16_t length);

    inline uint8_t getStation(){ return *station; }
    inline uint8_t getFunctionCode() { return *(station+1); }
    inline uint16_t getCRC(){ crc = (uint16_t*)(pack->endOfPack); return *crc; }
};

/*485数据包必须立刻使用*/
/****************诊断包0x80+功能码****************/
class MBPDiagnose : public ModbusBasePack {
public:
  static const uint8_t FunctionCode = 0x80;

  static const uint8_t DiagnoseCode_NoError = 0x00;
  static const uint8_t DiagnoseCode_InvalidFunctionCode = 0x01;
  static const uint8_t DiagnoseCode_InvalidDataAddress = 0x02;
  static const uint8_t DiagnoseCode_InvalidDataValue = 0x03;
  static const uint8_t DiagnoseCode_SlaveDeviceFault = 0x04;
  static const uint8_t DiagnoseCode_SlaveExecuting = 0x05;
  static const uint8_t DiagnoseCode_SlaveBusy = 0x06;
  static const uint8_t DiagnoseCode_CRCFailed = 0x08;
  static const uint8_t DiagnoseCode_BadGateway = 0x0A;
  static const uint8_t DiagnoseCode_SlaveNoResponse = 0x0B;

  uint8_t *diagnoseCode;
  uint8_t *cast(uint8_t *buf, bool isNew = false);
  void write(Stream &s);
  inline void setDiagnoseCode(uint8_t code){
    *diagnoseCode = code;
  }
  inline uint8_t getDiagnoseCode(uint8_t code){
    return *diagnoseCode;
  }
};
/****************读线圈寄存器0x01****************/
//请求
class MBPReadCoilRegisterRequest : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x01;
  uint16_modbus *startAddress;
  uint16_modbus *quantity;
  uint8_t *cast(uint8_t *buf, bool isNew = false);
  void write(Stream &s);
  inline uint16_t getStartAddress(){ return startAddress->get(); }
  inline uint16_t getQuantity(){ return quantity->get(); }
  inline void setStartAddress(uint16_t address) { startAddress->set(address); }
  inline void setQuantity(uint16_t quant) { quantity->set(quant); }
  void popRegisters(bool toTail, uint16_t quant);
};
//回复
class MBPReadCoilRegisterResponse : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x01;
  uint8_t *bytes;
  uint8_t *values;
  uint16_t _quantity;
  uint8_t *cast(uint8_t *buf, bool isNew = false);
  void write(Stream &s);
  inline uint8_t getBytes(){ return *bytes; }
  inline void initValues(uint16_t quant){
    _quantity = quant;
    *bytes = (uint8_t)((quant+7)>>3);
    for(uint8_t i = 0;i<*bytes;i++) values[i] = 0;
    setEOP(((uint8_t*)values)+getBytes());
  }
  inline void setValue(uint8_t atAddress, bool state) { 
    uint8_t bitBlock = (atAddress>>3);
    if(bitBlock >= getBytes()) return;
    uint8_t bitIndex = atAddress&0x07;
    values[bitBlock] &= ~(1 << bitIndex);
    values[bitBlock] |= (state << bitIndex);
  }
  inline bool getValue(uint8_t atAddress){
    uint8_t bitBlock = (atAddress>>3);
    if(bitBlock >= getBytes()) return 0;
    uint8_t bitIndex = atAddress&0x07;
    return (values[bitBlock] >> bitIndex)&0x01;
  }
  inline void addValue(bool state){
    uint16_t vIndex = _quantity;
    _quantity++;
    *bytes = (uint8_t)((_quantity+7)>>3);
    setValue(vIndex,state);
    setEOP(((uint8_t*)values)+getBytes());
  }
  void pushRegisters(bool toTail, uint16_t quant, uint8_t *data);
};

/****************读离散输入寄存器0x02****************/
//请求
class MBPReadDiscreteInputRegisterRequest : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x02;
  uint16_modbus *startAddress;
  uint16_modbus *quantity;
  uint8_t *cast(uint8_t *buf, bool isNew = false);
  void write(Stream &s);
  inline uint16_t getStartAddress(){ return startAddress->get(); }
  inline uint16_t getQuantity(){ return quantity->get(); }
  inline void setStartAddress(uint16_t address) { startAddress->set(address); }
  inline void setQuantity(uint16_t quant) { quantity->set(quant); }
  void popRegisters(bool toTail, uint16_t quant);
};
//回复
class MBPReadDiscreteInputRegisterResponse : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x02;
  uint8_t *bytes;
  uint8_t *values;
  uint16_t _quantity;
  uint8_t *cast(uint8_t *buf, bool isNew = false);
  void write(Stream &s);
  inline uint8_t getBytes(){ return *bytes; }
  inline void initValues(uint16_t quant){
    _quantity = quant;
    *bytes = (uint8_t)((quant+7)>>3);
    for(uint8_t i = 0;i<*bytes;i++) values[i] = 0;
    setEOP(((uint8_t*)values)+getBytes());
  }
  inline void setValue(uint8_t atAddress, bool state) { 
    uint8_t bitBlock = (atAddress>>3);
    if(bitBlock >= getBytes()) return;
    uint8_t bitIndex = atAddress&0x07;
    values[bitBlock] &= ~(1 << bitIndex);
    values[bitBlock] |= (state << bitIndex);
  }
  inline bool getValue(uint8_t atAddress){
    uint8_t bitBlock = (atAddress>>3);
    if(bitBlock >= getBytes()) return 0;
    uint8_t bitIndex = atAddress&0x07;
    return (values[bitBlock] >> bitIndex)&0x01;
  }
  inline void addValue(bool state){
    uint16_t vIndex = _quantity;
    _quantity++;
    *bytes = (uint8_t)((_quantity+7)>>3);
    setValue(_quantity,state);
    setEOP(((uint8_t*)values)+getBytes());
  }
  void pushRegisters(bool toTail, uint16_t quant, uint8_t *data);
};

/****************读保持寄存器0x03****************/
//请求 (和 读线圈寄存器 基本一致)
class MBPReadHoldingRegisterRequest : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x03;
  uint16_modbus *startAddress;
  uint16_modbus *quantity;
  uint8_t *cast(uint8_t *buf, bool isNew = false);
  void write(Stream &s);
  inline uint16_t getStartAddress(){ return startAddress->get(); }
  inline uint16_t getQuantity(){ return quantity->get(); }
  inline void setStartAddress(uint16_t address) { startAddress->set(address); }
  inline void setQuantity(uint16_t quant) { quantity->set(quant); }
  void popRegisters(bool toTail, uint16_t quant);
};
//回复
class MBPReadHoldingRegisterResponse : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x03;
  uint8_t *bytes;
  uint16_modbus *values;
  uint16_t _quantity;
  uint8_t *cast(uint8_t *buf, bool isNew = false);
  void write(Stream &s);
  inline uint8_t getBytes(){ return *bytes; }
  inline void initValues(uint16_t quant){
    *bytes = (uint8_t)(quant*2);
    for(uint8_t i = 0;i<*bytes;i++) values[i] = 0;
    setEOP(((uint8_t*)values)+getBytes());
    _quantity = quant;
  }
  inline void setValue(uint8_t atAddress, uint16_t data) { 
    if(atAddress >= getBytes()/2) return;
    values[atAddress].set(data);
  }
  inline uint16_t getValue(uint8_t atAddress){
    if(atAddress >= getBytes()/2) return 0;
    return values[atAddress].get();
  }
  inline void addValue(uint16_t state){
    uint16_t vIndex = _quantity;
    _quantity++;
    *bytes = (uint8_t)(_quantity*2);
    setValue(vIndex,state);
    setEOP(((uint8_t*)values)+getBytes());
  }
  void pushRegisters(bool toTail, uint16_t quant, uint8_t *data);
};

/****************读输入寄存器0x04****************/
//请求 (和 读保持寄存器 基本一致)
class MBPReadInputRegisterRequest : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x04;
  uint16_modbus *startAddress;
  uint16_modbus *quantity;
  uint8_t *cast(uint8_t *buf, bool isNew = false);
  void write(Stream &s);
  inline uint16_t getStartAddress(){ return startAddress->get(); }
  inline uint16_t getQuantity(){ return quantity->get(); }
  inline void setStartAddress(uint16_t address) { startAddress->set(address); }
  inline void setQuantity(uint16_t quant) { quantity->set(quant); }
  void popRegisters(bool toTail, uint16_t quant);
};
//回复
class MBPReadInputRegisterResponse : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x04;
  uint8_t *bytes;
  uint16_modbus *values;
  uint16_t _quantity;
  uint8_t *cast(uint8_t *buf, bool isNew = false);
  void write(Stream &s);
  inline uint8_t getBytes(){ return *bytes; }
  inline void initValues(uint16_t quant){
    *bytes = (uint8_t)(quant*2);
    for(uint8_t i = 0;i<*bytes;i++) values[i] = 0;
    setEOP(((uint8_t*)values)+getBytes());
    _quantity = quant;
  }
  inline void setValue(uint8_t atAddress, uint16_t data) { 
    if(atAddress >= getBytes()/2) return;
    values[atAddress].set(data);
  }
  inline uint16_t getValue(uint8_t atAddress){
    if(atAddress >= getBytes()/2) return 0;
    return values[atAddress].get();
  }
  inline void addValue(uint16_t state){
    uint16_t vIndex = _quantity;
    _quantity++;
    *bytes = (uint8_t)(_quantity*2);
    setValue(vIndex,state);
    setEOP(((uint8_t*)values)+getBytes());
  }
  void pushRegisters(bool toTail, uint16_t quant, uint8_t *data);
};

/****************写单个线圈寄存器0x05****************/
//请求
class MBPWriteCoilRegisterRequest : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x05;
  uint16_modbus *startAddress;
  uint16_modbus *value;
  uint8_t *cast(uint8_t *buf, bool isNew = false);
  void write(Stream &s);
  inline uint16_t getStartAddress(){ return startAddress->get(); }
  inline void setStartAddress(uint16_t address) { startAddress->set(address); }
  inline void setValue(bool data) { value->set(data?0xFF00:0x0000); }
  inline bool getValue() { return value->get() == 0xFF00; }
};
//回复 ( 与请求报文一样 )
class MBPWriteCoilRegisterResponse : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x05;
  uint16_modbus *startAddress;
  uint16_modbus *value;
  uint8_t *cast(uint8_t *buf, bool isNew = false);
  void write(Stream &s);
  inline uint16_t getStartAddress(){ return startAddress->get(); }
  inline void setStartAddress(uint16_t address) { startAddress->set(address); }
  inline void setValue(bool data) { value->set(data?0xFF00:0x0000); }
  inline bool getValue() { return value->get() == 0xFF00; }
};

/****************写单个保持寄存器0x06****************/
//请求
class MBPWriteHoldingRegisterRequest : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x06;
  uint16_modbus *startAddress;
  uint16_modbus *value;
  uint8_t *cast(uint8_t *buf, bool isNew = false);
  void write(Stream &s);
  inline uint16_t getStartAddress(){ return startAddress->get(); }
  inline void setStartAddress(uint16_t address) { startAddress->set(address); }
  inline void setValue(uint16_t data) { value->set(data); }
  inline uint16_t getValue() { return value->get(); }
};
//回复 ( 与请求报文一样 )
class MBPWriteHoldingRegisterResponse : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x06;
  uint16_modbus *startAddress;
  uint16_modbus *value;
  uint8_t *cast(uint8_t *buf, bool isNew = false);
  void write(Stream &s);
  inline uint16_t getStartAddress(){ return startAddress->get(); }
  inline void setStartAddress(uint16_t address) { startAddress->set(address); }
  inline void setValue(uint16_t data) { value->set(data); }
  inline uint16_t getValue() { return value->get(); }
};

/****************写多个线圈寄存器0x0F****************/
//请求
class MBPWriteMultipleCoilRegistersRequest : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x0F;
  uint16_modbus *startAddress;
  uint16_modbus *quantity;
  uint8_t *bytes;
  uint8_t *values;
  uint8_t *cast(uint8_t *buf, bool isNew = false);
  void write(Stream &s);
  inline uint16_t getStartAddress(){ return startAddress->get(); }
  inline uint16_t getQuantity(){ return quantity->get(); }
  inline void setStartAddress(uint16_t address) { startAddress->set(address); }
  inline void setQuantity(uint16_t quant) { quantity->set(quant); *bytes = (uint8_t)((quant+7)>>3); }
  inline uint8_t getBytes(){ return *bytes; }
  inline void initValues(uint16_t quant){
    setQuantity(quant);
    for(uint8_t i = 0;i<*bytes;i++) values[i] = 0;
    setEOP(((uint8_t*)values)+getBytes());
  }
  inline void setValue(uint8_t atAddress, bool state) { 
    uint8_t bitBlock = (atAddress>>3);
    if(bitBlock >= getBytes()) return;
    uint8_t bitIndex = atAddress&0x07;
    values[bitBlock] &= ~(1 << bitIndex);
    values[bitBlock] |= (state << bitIndex);
  }
  inline bool getValue(uint8_t atAddress){
    uint8_t bitBlock = (atAddress>>3);
    if(bitBlock >= getBytes()) return 0;
    uint8_t bitIndex = atAddress&0x07;
    return (values[bitBlock] >> bitIndex)&0x01;
  }
  inline void addValue(bool state){
    uint16_t vIndex = getQuantity();
    setQuantity(vIndex+1);
    setValue(vIndex,state);
    setEOP(((uint8_t*)values)+getBytes());
  }
  void pushRegisters(bool toTail, uint16_t quant, uint8_t *data);
  void popRegisters(bool toTail, uint16_t quant);
};
//回复
class MBPWriteMultipleCoilRegistersResponse : public ModbusBasePack{
public:
  static const uint8_t FunctionCode = 0x0F;
  uint16_modbus *startAddress;
  uint16_modbus *quantity;
  uint8_t *cast(uint8_t *buf, bool isNew = false);
  void write(Stream &s);
  inline uint16_t getStartAddress(){ return startAddress->get(); }
  inline uint16_t getQuantity(){ return quantity->get(); }
  inline void setStartAddress(uint16_t address) { startAddress->set(address); }
  inline void setQuantity(uint16_t quant) { quantity->set(quant); }
  void pushRegisters(bool toTail, uint16_t quant, uint8_t *data);
};

/****************写多个保持寄存器0x10****************/
//请求
class MBPWriteMultipleHoldingRegistersRequest : public MBPWriteMultipleCoilRegistersRequest{
public:
  static const uint8_t FunctionCode = 0x10;
  uint16_modbus *startAddress;
  uint16_modbus *quantity;
  uint8_t *bytes;
  uint16_modbus *values;
  uint8_t *cast(uint8_t *buf, bool isNew = false);
  void write(Stream &s);  inline uint16_t getStartAddress(){ return startAddress->get(); }
  inline uint16_t getQuantity(){ return quantity->get(); }
  inline void setStartAddress(uint16_t address) { startAddress->set(address); }
  inline void setQuantity(uint16_t quant) { quantity->set(quant); *bytes = quant*2; }
  inline uint8_t getBytes(){ return *bytes; }
  inline void initValues(uint16_t quant){
    setQuantity(quant);
    *bytes = (uint8_t)(quant*2);
    for(uint8_t i = 0;i<*bytes;i++) values[i] = 0;
    setEOP(((uint8_t*)values)+getBytes());
  }
  inline void setValue(uint8_t atAddress, uint16_t data) { 
    if(atAddress >= getQuantity()) return;
    values[atAddress].set(data);
  }
  inline uint16_t getValue(uint8_t atAddress){
    if(atAddress >= getQuantity()) return 0;
    return values[atAddress].get();
  }
  inline void addValue(uint16_t state){
    uint16_t vIndex = getQuantity();
    setQuantity(vIndex+1);
    setValue(vIndex,state);
    setEOP(((uint8_t*)values)+getBytes());
  }
  void popRegisters(bool toTail, uint16_t quant);
  void pushRegisters(bool toTail, uint16_t quant, uint8_t *data);
};
//回复
class MBPWriteMultipleHoldingRegistersResponse : public MBPWriteMultipleCoilRegistersResponse{
public:
  static const uint8_t FunctionCode = 0x10;
  uint16_modbus *startAddress;
  uint16_modbus *quantity;
  uint8_t *cast(uint8_t *buf, bool isNew = false);
  void write(Stream &s);
  inline uint16_t getStartAddress(){ return startAddress->get(); }
  inline uint16_t getQuantity(){ return quantity->get(); }
  inline void setStartAddress(uint16_t address) { startAddress->set(address); }
  inline void setQuantity(uint16_t quant) { quantity->set(quant); }
  void pushRegisters(bool toTail, uint16_t quant, uint8_t *data);
};

#pragma pack(pop)

