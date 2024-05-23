#pragma once
#include "Arduino.h"
#include "ModbusPack.h"
#define ModbusRegisterConfigTemplate size_t pbCoilCount, size_t bCoilCount, size_t pbDiscreteInputCount, size_t bDiscreteInputCount, size_t pwInputCount, size_t wInputCount, size_t pwHoldCount, size_t wHoldCount
#define ModbusRegisterConfigArgs pbCoilCount, bCoilCount, pbDiscreteInputCount, bDiscreteInputCount, pwInputCount, wInputCount, pwHoldCount, wHoldCount

template<ModbusRegisterConfigTemplate>
class ModbusRegister {
private:
public:
    typedef void(*ModbusRegisterSetCallback)(ModbusRegister *reg, uint16_t address);
    typedef bool(*ModbusRegisterGetCallback)(ModbusRegister *reg, uint16_t address, uint16_t &data);
    typedef void(*ModbusOnCustomProcess)(ModbusFrame &packIn, ModbusFrame &packOut);

    uint8_t* pbCoil[(pbCoilCount+7) / 8];    //输出线圈
    uint8_t* pbDiscreteInput[(pbDiscreteInputCount+7) / 8];    //输入触点
    uint16_t* pwInput[pwInputCount];   //输入数字量
    uint16_t* pwHold[pwInputCount];   //输入数字量
    uint8_t bCoil[(bCoilCount+7) / 8];     //输出线圈
    uint8_t bDiscreteInput[(bDiscreteInputCount+7) / 8];    //输入触点
    uint16_t wInput[wInputCount];   //输入数字量
    uint16_t wHold[wHoldCount];    //保持数字量
public:
    ModbusRegister();
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
    inline void setHoldFast(uint16_t address, uint16_t data);
    inline void getHoldFast(uint16_t address, uint16_t &data);
    uint8_t *getCoilPointer(uint16_t address);
    uint8_t *getDiscreteInputPointer(uint16_t address);
    uint16_t *getInputPointer(uint16_t address);
    uint16_t *getHoldPointer(uint16_t address);
    uint16_t &getInputRef(uint16_t address);   //Only non-pointer register
    uint16_t &getHoldRef(uint16_t address);   //Only non-pointer register

    ModbusRegisterGetCallback onHoldGet;
    ModbusRegisterSetCallback onHoldSet;
    ModbusRegisterGetCallback onCoilGet;
    ModbusRegisterSetCallback onCoilSet;
    ModbusRegisterGetCallback onDiscreteInputGet;
    ModbusRegisterGetCallback onInputGet;

    void registerCoil(uint16_t address, uint8_t *memAddress);
    void registerDiscreteInput(uint16_t address, uint8_t *memAddress);
    void registerInput(uint16_t address, uint16_t *memAddress);
    void registerHold(uint16_t address, uint16_t *memAddress);

    uint8_t process(ModbusFrame &packIn, ModbusFrame &packOut);
};


template<ModbusRegisterConfigTemplate>
ModbusRegister<ModbusRegisterConfigArgs>::ModbusRegister() {
    onHoldGet = 0;
    onHoldSet = 0;
    onCoilGet = 0;
    onCoilSet = 0;
    onDiscreteInputGet = 0;
    onInputGet = 0;
}

template<ModbusRegisterConfigTemplate>
void ModbusRegister<ModbusRegisterConfigArgs>::registerCoil(uint16_t address, uint8_t *memAddress){   //一次必须映射8个线圈
    if(address < pbCoilCount) pbCoil[address] = memAddress;
}

template<ModbusRegisterConfigTemplate>
void ModbusRegister<ModbusRegisterConfigArgs>::registerDiscreteInput(uint16_t address, uint8_t *memAddress){  //一次必须映射8个输入状态
    if(address < pbDiscreteInputCount) pbDiscreteInput[address] = memAddress;
}

template<ModbusRegisterConfigTemplate>
void ModbusRegister<ModbusRegisterConfigArgs>::registerInput(uint16_t address, uint16_t *memAddress){
    if(address < pwInputCount) pwInput[address] = memAddress;
}

template<ModbusRegisterConfigTemplate>
void ModbusRegister<ModbusRegisterConfigArgs>::registerHold(uint16_t address, uint16_t *memAddress){
    if(address < pwHoldCount) pwHold[address] = memAddress;
}

template<ModbusRegisterConfigTemplate>
uint8_t ModbusRegister<ModbusRegisterConfigArgs>::setCoil(uint16_t address, bool state){
    if(address < pbCoilCount){
        if(pbCoil == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Coil Register Pointer
        uint16_t bitBlock = address/8;
        uint8_t bitIndex = address%8;
        *(pbCoil[bitBlock]) &= ~(1 << bitIndex);
        *(pbCoil[bitBlock]) |= (state << bitIndex);
        if(onCoilSet) onCoilSet(this,address);
    }else if(address < pbCoilCount+bCoilCount){
        if(bCoil == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Coil Register
        uint16_t npAddress = address-pbCoilCount;
        uint16_t bitBlock = npAddress/8;
        uint8_t bitIndex = npAddress%8;
        bCoil[bitBlock] &= ~(1 << bitIndex);
        bCoil[bitBlock] |= (state << bitIndex);
        if(onCoilSet) onCoilSet(this,address);
    }else{
        return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    }
    return 0;
}

template<ModbusRegisterConfigTemplate>
uint8_t *ModbusRegister<ModbusRegisterConfigArgs>::getCoilPointer(uint16_t address){
    if(address < pbCoilCount){
        uint16_t bitBlock = address/8;
        uint8_t bitIndex = address%8;
        return pbCoil[bitBlock];
    }else if(address < pbCoilCount+bCoilCount){
        uint16_t npAddress = address-pbCoilCount;
        uint16_t bitBlock = npAddress/8;
        uint8_t bitIndex = npAddress%8;
        return &(bCoil[bitBlock]);
    }
    return 0;
}

template<ModbusRegisterConfigTemplate>
uint8_t ModbusRegister<ModbusRegisterConfigArgs>::getCoil(uint16_t address, bool &state){
    if(address < pbCoilCount){
        if(pbCoil == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Coil Register Pointer
        uint16_t bitBlock = address/8;
        uint8_t bitIndex = address%8;
        uint16_t u16State = state;
        if(onCoilGet && onCoilGet(this,address,u16State))
            state = u16State;
        else
            state = (*(pbCoil[bitBlock]) >> bitIndex)&0x01;
    }else if(address < pbCoilCount+bCoilCount){
        if(bCoil == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Coil Register
        uint16_t npAddress = address-pbCoilCount;
        uint16_t bitBlock = npAddress/8;
        uint8_t bitIndex = npAddress%8;
        uint16_t u16State = state;
        if(onCoilGet && onCoilGet(this,address,u16State))
            state = u16State;
        else
            state = (bCoil[bitBlock] >> bitIndex)&0x01;
    }else{
        return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    }
    return 0;
}

template<ModbusRegisterConfigTemplate>
bool ModbusRegister<ModbusRegisterConfigArgs>::getCoil(uint16_t address){
    bool state = false;
    this->getCoil(address, state);
    return state;
}

template<ModbusRegisterConfigTemplate>
uint8_t *ModbusRegister<ModbusRegisterConfigArgs>::getDiscreteInputPointer(uint16_t address){
    if(address < pbDiscreteInputCount){
        uint16_t bitBlock = address/8;
        uint8_t bitIndex = address%8;
        return pbDiscreteInput[bitBlock];
    }else if(address < pbDiscreteInputCount+bDiscreteInputCount){
        uint16_t npAddress = address-pbDiscreteInputCount;
        uint16_t bitBlock = npAddress/8;
        uint8_t bitIndex = npAddress%8;
        return &(bDiscreteInput[bitBlock]);
    }
    return 0;
}

template<ModbusRegisterConfigTemplate>
uint8_t ModbusRegister<ModbusRegisterConfigArgs>::getDiscreteInput(uint16_t address, bool &state){
    if(address < pbDiscreteInputCount){
        if(pbDiscreteInput == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Discrete Input Register Pointer
        uint16_t bitBlock = address/8;
        uint8_t bitIndex = address%8;
        uint16_t u16State = state;
        if(onDiscreteInputGet && onDiscreteInputGet(this,address,u16State))
            state = u16State;
        else
            state = (*(pbDiscreteInput[bitBlock]) >> bitIndex)&0x01;
    }else if(address < pbDiscreteInputCount+bDiscreteInputCount){
        if(bDiscreteInput == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Discrete Input Register
        uint16_t npAddress = address-pbDiscreteInputCount;
        uint16_t bitBlock = npAddress/8;
        uint8_t bitIndex = npAddress%8;
        uint16_t u16State = state;
        if(onDiscreteInputGet && onDiscreteInputGet(this,address,u16State))
            state = u16State;
        else
            state = (bDiscreteInput[bitBlock] >> bitIndex)&0x01;
    }else{
        return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    }
    return 0;
}

template<ModbusRegisterConfigTemplate>
bool ModbusRegister<ModbusRegisterConfigArgs>::getDiscreteInput(uint16_t address){
    bool state = false;
    this->getDiscreteInput(address, state);
    return state;
}


template<ModbusRegisterConfigTemplate>
uint16_t *ModbusRegister<ModbusRegisterConfigArgs>::getInputPointer(uint16_t address){
    if(address < pwInputCount){
        return pwInput[address];
    }else if(address < pwInputCount+wInputCount){
        uint16_t npAddress = address-pwInputCount;
        return &(wInput[npAddress]);
    }
    return 0;
}

template<ModbusRegisterConfigTemplate>
uint16_t &ModbusRegister<ModbusRegisterConfigArgs>::getInputRef(uint16_t address){   //Only non-pointer register
    if(address < pwInputCount+wInputCount)
        return wHold[address-pwInputCount];
    return wInput[0];
}

template<ModbusRegisterConfigTemplate>
uint8_t ModbusRegister<ModbusRegisterConfigArgs>::getInput(uint16_t address, uint16_t &data){
    if(address < pwInputCount){
        if(pwInput == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Input Register Pointer
        if(!(onInputGet && onInputGet(this,address,data)))
            data = *(pwInput[address]);
    }else if(address < pwInputCount+wInputCount){
        if(wInput == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Input Register
        if(!(onInputGet && onInputGet(this,address,data))){
            uint16_t npAddress = address-pwInputCount;
            data = wInput[npAddress];
        }

    }else{
        return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    }
    return 0;
}

template<ModbusRegisterConfigTemplate>
uint16_t ModbusRegister<ModbusRegisterConfigArgs>::getInput(uint16_t address){
    uint16_t state = 0;
    this->getInput(address, state);
    return state;
}

template<ModbusRegisterConfigTemplate>
uint8_t ModbusRegister<ModbusRegisterConfigArgs>::setHold(uint16_t address, uint16_t data){
    if(address < pwHoldCount){
        if(pwHold == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Input Register Pointer
        *(pwHold[address]) = data;
        if(onHoldSet) onHoldSet(this,address);
    }else if(address < pwHoldCount+wHoldCount){
        if(wHold == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Input Register
        uint16_t npAddress = address-pwHoldCount;
        wHold[npAddress] = data;
        if(onHoldSet) onHoldSet(this,address);
    }else{
        return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    }
    return 0;
}

template<ModbusRegisterConfigTemplate>
inline void ModbusRegister<ModbusRegisterConfigArgs>::setHoldFast(uint16_t address, uint16_t data){
    if(address < pwHoldCount){
        *(pwHold[address]) = data;
    }else if(address < pwHoldCount+wHoldCount){
        uint16_t npAddress = address-pwHoldCount;
        wHold[npAddress] = data;
    }
}

template<ModbusRegisterConfigTemplate>
uint16_t *ModbusRegister<ModbusRegisterConfigArgs>::getHoldPointer(uint16_t address){
    if(address < pwHoldCount){
        return pwHold[address];
    }else if(address < pwHoldCount+wHoldCount){
        uint16_t npAddress = address-pwHoldCount;
        return &(wHold[npAddress]);
    }
    return 0;
}

template<ModbusRegisterConfigTemplate>
uint16_t &ModbusRegister<ModbusRegisterConfigArgs>::getHoldRef(uint16_t address){   //Only non-pointer register
    if(address < pwHoldCount+wHoldCount)
        return wHold[address-pwHoldCount];
    return wHold[0];
}

template<ModbusRegisterConfigTemplate>
uint8_t ModbusRegister<ModbusRegisterConfigArgs>::getHold(uint16_t address, uint16_t &data){
    if(address < pwHoldCount){
        if(pwHold == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Input Register Pointer
        if(!(onHoldGet && onHoldGet(this,address,data)))
            data = *(pwHold[address]);
    }else if(address < pwHoldCount+wHoldCount){
        if(wHold == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Input Register
        if(!(onHoldGet && onHoldGet(this,address,data))){
            uint16_t npAddress = address-pwHoldCount;
            data = wHold[npAddress];
        }
    }else{
        return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    }
    return 0;
}

template<ModbusRegisterConfigTemplate>
uint16_t ModbusRegister<ModbusRegisterConfigArgs>::getHold(uint16_t address){
    uint16_t state = 0;
    this->getHold(address, state);
    return state;
}


template<ModbusRegisterConfigTemplate>
inline void ModbusRegister<ModbusRegisterConfigArgs>::getHoldFast(uint16_t address, uint16_t &data){
    if(address < pwHoldCount){
        data = *(pwHold[address]);
    }else if(address < pwHoldCount+wHoldCount){
        uint16_t npAddress = address-pwHoldCount;
        data = wHold[npAddress];
    }
}

template<ModbusRegisterConfigTemplate>
uint8_t ModbusRegister<ModbusRegisterConfigArgs>::process(ModbusFrame &frameIn, ModbusFrame &frameOut){
    uint8_t result = 0;
    if(!frameOut.createResponse(frameIn.pack->getFunctionCode())) return 0;
    switch(frameIn.pack->getFunctionCode()){
    case MBPReadCoilRegisterRequest::FunctionCode: {
        MBPReadCoilRegisterRequest *pIn = (MBPReadCoilRegisterRequest *)(frameIn.pack);
        MBPReadCoilRegisterResponse *pOut = (MBPReadCoilRegisterResponse *)(frameOut.pack);
        #ifdef DEBUG_MODBUS_ON
        Serial.println("读线圈");
        #endif
        pOut->initValues(pIn->getQuantity());
        uint16_t startAddress = pIn->getStartAddress();
        for(uint16_t i=0; i<pIn->getQuantity(); i++){
            bool state = false;
            result = this->getCoil(startAddress+i,state);
            if(result != 0) break;
            pOut->setValue(i,state);
        }
        break;
    }
    case MBPReadDiscreteInputRegisterRequest::FunctionCode: {
        MBPReadDiscreteInputRegisterRequest *pIn = (MBPReadDiscreteInputRegisterRequest *)(frameIn.pack);
        MBPReadDiscreteInputRegisterResponse *pOut = (MBPReadDiscreteInputRegisterResponse *)(frameOut.pack);
        #ifdef DEBUG_MODBUS_ON
        Serial.println("读离散输入");
        #endif
        pOut->initValues(pIn->getQuantity());
        uint16_t startAddress = pIn->getStartAddress();
        for(uint16_t i=0; i<pIn->getQuantity(); i++){
            bool state = false;
            result = this->getDiscreteInput(startAddress+i,state);
            if(result != 0) break;
            pOut->setValue(i,state);
        }
        break;
    }
    case MBPReadHoldingRegisterRequest::FunctionCode: {
        MBPReadHoldingRegisterRequest *pIn = (MBPReadHoldingRegisterRequest *)(frameIn.pack);
        MBPReadHoldingRegisterResponse *pOut = (MBPReadHoldingRegisterResponse *)(frameOut.pack);
        #ifdef DEBUG_MODBUS_ON
        Serial.println("读保持寄存器");
        #endif
        pOut->initValues(pIn->getQuantity());
        uint16_t startAddress = pIn->getStartAddress();
        for(uint16_t i=0; i<pIn->getQuantity(); i++){
            uint16_t state = false;
            result = this->getHold(startAddress+i,state);
            if(result != 0) break;
            pOut->setValue(i,state);
        }
        break;
    }
    case MBPReadInputRegisterRequest::FunctionCode: {
        MBPReadInputRegisterRequest *pIn = (MBPReadInputRegisterRequest *)(frameIn.pack);
        MBPReadInputRegisterResponse *pOut = (MBPReadInputRegisterResponse *)(frameOut.pack);
        #ifdef DEBUG_MODBUS_ON
        Serial.println("读输入寄存器");
        #endif
        pOut->initValues(pIn->getQuantity());
        uint16_t startAddress = pIn->getStartAddress();
        for(uint16_t i=0; i<pIn->getQuantity(); i++){
            uint16_t state = false;
            result = this->getInput(startAddress+i,state);
            if(result != 0) break;
            pOut->setValue(i,state);
        }
        break;
    }
    case MBPWriteCoilRegisterRequest::FunctionCode: {
        MBPWriteCoilRegisterRequest *pIn = (MBPWriteCoilRegisterRequest *)(frameIn.pack);
        MBPWriteCoilRegisterResponse *pOut = (MBPWriteCoilRegisterResponse *)(frameOut.pack);
        uint16_t startAddress = pIn->getStartAddress();
        #ifdef DEBUG_MODBUS_ON
        Serial.println("写单线圈开始");
        Serial.print(startAddress);
        Serial.print(":");
        Serial.println(pIn->getValue());
        Serial.println(pIn->value->get());
        Serial.println("写单线圈结束");
        #endif
        result = this->setCoil(startAddress,pIn->getValue());
        if(result != 0) break;
        pOut->setValue(pIn->getValue());
        pOut->setStartAddress(pIn->getStartAddress());
        break;
    }
    case MBPWriteHoldingRegisterRequest::FunctionCode: {
        MBPWriteHoldingRegisterRequest *pIn = (MBPWriteHoldingRegisterRequest *)(frameIn.pack);
        MBPWriteHoldingRegisterResponse *pOut = (MBPWriteHoldingRegisterResponse *)(frameOut.pack);
        uint16_t startAddress = pIn->getStartAddress();
        #ifdef DEBUG_MODBUS_ON
        Serial.println("写单保持寄存器开始");
        Serial.print(startAddress);
        Serial.print(":");
        Serial.println(pIn->getValue());
        Serial.println("写单保持寄存器结束");
        #endif
        result = this->setHold(startAddress,pIn->getValue());
        if(result != 0) break;
        pOut->setValue(pIn->getValue());
        pOut->setStartAddress(pIn->getStartAddress());
        break;
    }
    case MBPWriteMultipleCoilRegistersRequest::FunctionCode: {
        MBPWriteMultipleCoilRegistersRequest *pIn = (MBPWriteMultipleCoilRegistersRequest *)(frameIn.pack);
        MBPWriteMultipleCoilRegistersResponse *pOut = (MBPWriteMultipleCoilRegistersResponse *)(frameOut.pack);
        uint16_t startAddress = pIn->getStartAddress();
        if((pIn->getQuantity()+7)/8 != pIn->getBytes()){
            result = MBPDiagnose::DiagnoseCode_InvalidDataValue;
            break;
        }
        #ifdef DEBUG_MODBUS_ON
        Serial.println("写多线圈开始");
        #endif
        for(uint16_t i=0; i<pIn->getQuantity(); i++){
            result = this->setCoil(startAddress+i,pIn->getValue(i));
            #ifdef DEBUG_MODBUS_ON
            Serial.print(startAddress+i);
            Serial.print(":");
            Serial.println(pIn->getValue(i));
            #endif
            if(result != 0) break;
        }
        #ifdef DEBUG_MODBUS_ON
        Serial.println("写多线圈结束");
        #endif
        pOut->setStartAddress(pIn->getStartAddress());
        pOut->setQuantity(pIn->getQuantity());
        break;
    }
    case MBPWriteMultipleHoldingRegistersRequest::FunctionCode: {
        MBPWriteMultipleHoldingRegistersRequest *pIn = (MBPWriteMultipleHoldingRegistersRequest *)(frameIn.pack);
        MBPWriteMultipleHoldingRegistersResponse *pOut = (MBPWriteMultipleHoldingRegistersResponse *)(frameOut.pack);
        uint16_t startAddress = pIn->getStartAddress();
        if(pIn->getQuantity()*2 != pIn->getBytes()){
            result = MBPDiagnose::DiagnoseCode_InvalidDataValue;
            break;
        }
        #ifdef DEBUG_MODBUS_ON
        Serial.println("写多保持寄存器开始");
        #endif
        for(uint16_t i=0; i<pIn->getQuantity(); i++){
            result = this->setHold(startAddress+i,pIn->getValue(i));
            #ifdef DEBUG_MODBUS_ON
            Serial.print(startAddress+i);
            Serial.print(":");
            Serial.println(pIn->getValue(i));
            #endif
            if(result != 0) break;
        }
        #ifdef DEBUG_MODBUS_ON
        Serial.println("写多保持寄存器结束");
        #endif
        pOut->setStartAddress(startAddress);
        pOut->setQuantity(pIn->getQuantity());
        break;
    }
    default:
        break; 
    }
    if(result != 0){
        Serial.println("发送错误回包");
        frameOut.createDiagnose(frameIn.pack->getFunctionCode());
        MBPDiagnose *pOutDiag = (MBPDiagnose *)(frameOut.pack);
        pOutDiag->setDiagnoseCode(result);
    }
    return result;
}