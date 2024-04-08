#pragma once
#include "Arduino.h"
#include "ModbusPack.h"
class ModbusRegisterConfig{
public:
    uint16_t pbCoilCount;    //指针线圈
    uint16_t bCoilCount;    //线圈
    uint16_t pbDiscreteInputCount;
    uint16_t bDiscreteInputCount;
    uint16_t pwInputCount;
    uint16_t wInputCount;
    uint16_t pwHoldCount;
    uint16_t wHoldCount;
};


class ModbusRegister {
private:
public:
    typedef void(*ModbusRegisterCallback)(ModbusRegister *reg, uint16_t address);

    ModbusRegisterConfig regConf;
    uint8_t** pbCoil;    //输出线圈
    uint8_t** pbDiscreteInput;    //输入触点
    uint16_t** pwInput;   //输入数字量
    uint16_t** pwHold;    //保持数字量
    uint8_t* bCoil;     //输出线圈
    uint8_t* bDiscreteInput;    //输入触点
    uint16_t* wInput;   //输入数字量
    uint16_t* wHold;    //保持数字量
public:
    ModbusRegister(ModbusRegisterConfig regCount);
    ~ModbusRegister();
    uint8_t setCoil(uint16_t address, bool state);
    uint8_t getCoil(uint16_t address, bool &state);
    bool getCoil(uint16_t address);
    uint8_t getDiscreteInput(uint16_t address, bool &state);
    bool getDiscreteInput(uint16_t address);
    uint8_t getInput(uint16_t address, uint16_t &data);
    uint16_t getInput(uint16_t address);
    uint8_t setHold(uint16_t address, uint16_t data);
    uint8_t getHold(uint16_t address, uint16_t &data);
    uint16_t getHold(uint16_t address);

    ModbusRegisterCallback onHoldSet;
    ModbusRegisterCallback onCoilSet;
    void registerCoil(uint16_t address, uint8_t *memAddress);
    void registerDiscreteInput(uint16_t address, uint8_t *memAddress);
    void registerInput(uint16_t address, uint16_t *memAddress);
    void registerHold(uint16_t address, uint16_t *memAddress);

    uint8_t process(ModbusFrame &packIn, ModbusFrame &packOut);
};
