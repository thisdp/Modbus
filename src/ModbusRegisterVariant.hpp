#pragma once
#include "Arduino.h"
#include "ModbusPack.h"
#include <vector>
#include <stdexcept>
using namespace std;
// 使用 uint8_t 作为底层枚举类型的大小
enum class RegisterType : uint8_t { Direct, Pointer };

// 模板化的 RegVariant 类
template<typename T>
class RegVariant {
public:
    union DataUnion {
        T value;
        T* ptr;
    } data;

    RegisterType type;

    RegVariant() : data(), type(RegisterType::Direct) {}
    RegVariant(T v) : data(), type(RegisterType::Direct) { this->data.value = v; }
    RegVariant(T* p) : data(), type(RegisterType::Pointer) { this->data.ptr = p; }
    bool isDirect() const { return type == RegisterType::Direct; }
    bool isPointer() const { return type == RegisterType::Pointer; }
    T &get() {
        if (isDirect()) {
            return data.value;
        } else if (isPointer()) {
            if (!data.ptr) throw std::runtime_error("Null pointer");
            return *data.ptr;
        } else {
            throw std::runtime_error("Invalid register type");
        }
    }
    bool get(T &out) {
        if (isDirect()) {
            out = data.value;
            return true;
        } else if (isPointer()) {
            if (!data.ptr) return false;
            out = *data.ptr;
            return true;
        }
        return false;
    }
    bool set(T newValue) {
        if (isDirect()) {
            data.value = newValue;
            return true;
        } else if (isPointer()) {
            if (!data.ptr) return false;
            *data.ptr = newValue;
            return true;
        }
        return false;
    }
    void setAsDirect(T v) {
        type = RegisterType::Direct;
        data.value = v;
    }
    void setAsPointer(T* p) {
        type = RegisterType::Pointer;
        data.ptr = p;
    }
};

class ModbusRegisterVariant {
private:
    uint16_t emptyPointer;
public:
    typedef void(*ModbusRegisterVariantSetCallback)(ModbusRegisterVariant *reg, uint16_t address, uint16_t oldData);
    typedef bool(*ModbusRegisterVariantGetCallback)(ModbusRegisterVariant *reg, uint16_t address, uint16_t &data);
    typedef void(*ModbusOnCustomProcess)(ModbusFrame &packIn, ModbusFrame &packOut);

    vector<RegVariant<uint8_t>> bCoil;                 //输出线圈
    vector<RegVariant<uint8_t>> bDiscreteInput;        //输入触点
    vector<RegVariant<uint16_t>> wInput;               //输入数字量
    vector<RegVariant<uint16_t>> wHold;                //保持数字量
public:
    ModbusRegisterVariant(size_t bCoilCount, size_t bDiscreteInputCount, size_t wInputCount, size_t wHoldCount);
    uint8_t setCoil(uint16_t address, uint8_t state);
    uint8_t getCoil(uint16_t address, uint8_t &state);
    uint8_t getCoil(uint16_t address);
    uint8_t setDiscreteInput(uint16_t address, uint8_t state);
    uint8_t getDiscreteInput(uint16_t address, uint8_t &state);
    uint8_t getDiscreteInput(uint16_t address);
    uint8_t setInput(uint16_t address, uint16_t data);
    uint8_t getInput(uint16_t address, uint16_t &data);
    uint16_t getInput(uint16_t address);
    uint8_t setHold(uint16_t address, uint16_t data);
    uint8_t getHold(uint16_t address, uint16_t &data);
    uint16_t getHold(uint16_t address);
    inline void setHoldFast(uint16_t address, uint16_t data);
    inline void getHoldFast(uint16_t address, uint16_t &data);
    uint16_t &getInputRef(uint16_t address);   //Only non-pointer register
    uint16_t &getHoldRef(uint16_t address);    //Only non-pointer register

    ModbusRegisterVariantGetCallback onHoldGet;
    ModbusRegisterVariantSetCallback onHoldSet;
    ModbusRegisterVariantGetCallback onCoilGet;
    ModbusRegisterVariantSetCallback onCoilSet;
    ModbusRegisterVariantGetCallback onDiscreteInputGet;
    ModbusRegisterVariantSetCallback onDiscreteInputSet;
    ModbusRegisterVariantGetCallback onInputGet;
    ModbusRegisterVariantSetCallback onInputSet;

    uint8_t registerCoil(uint16_t address, uint8_t *memAddress);
    uint8_t registerDiscreteInput(uint16_t address, uint8_t *memAddress);
    uint8_t registerInput(uint16_t address, uint16_t *memAddress);
    uint8_t registerHold(uint16_t address, uint16_t *memAddress);
    uint8_t registerCoil(uint16_t address, uint8_t &memAddress);
    uint8_t registerDiscreteInput(uint16_t address, uint8_t &memAddress);
    uint8_t registerInput(uint16_t address, uint16_t &memAddress);
    uint8_t registerHold(uint16_t address, uint16_t &memAddress);

    uint8_t process(ModbusFrame &packIn, ModbusFrame &packOut);
    uint8_t processResponse(ModbusFrame &frameResponse, ModbusFrame &frameRequest);
};

ModbusRegisterVariant::ModbusRegisterVariant(size_t bCoilCount, size_t bDiscreteInputCount, size_t wInputCount, size_t wHoldCount) {
    bCoil.resize(bCoilCount);
    bDiscreteInput.resize(bDiscreteInputCount);
    wInput.resize(wInputCount);
    wHold.resize(wHoldCount);
    onHoldGet = 0;
    onHoldSet = 0;
    onCoilGet = 0;
    onCoilSet = 0;
    onDiscreteInputGet = 0;
    onInputGet = 0;
}

uint8_t ModbusRegisterVariant::registerCoil(uint16_t address, uint8_t *target){   //一次必须映射8个线圈
    if(address >= bCoil.size()) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    bCoil[address].setAsPointer(target);
    return 0;
}

uint8_t ModbusRegisterVariant::registerCoil(uint16_t address, uint8_t &target){   //一次必须映射8个线圈
    if(address >= bCoil.size()) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    bCoil[address].setAsPointer(&target);
    return 0;
}

uint8_t ModbusRegisterVariant::registerDiscreteInput(uint16_t address, uint8_t *target){  //一次必须映射8个输入状态
    if(address >= bDiscreteInput.size()) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    bDiscreteInput[address].setAsPointer(target);
    return 0;
}

uint8_t ModbusRegisterVariant::registerDiscreteInput(uint16_t address, uint8_t &target){  //一次必须映射8个输入状态
    if(address >= bDiscreteInput.size()) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    bDiscreteInput[address].setAsPointer(&target);
    return 0;
}

uint8_t ModbusRegisterVariant::registerInput(uint16_t address, uint16_t *target){
    if(address >= wInput.size()) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    wInput[address].setAsPointer(target);
    return 0;
}

uint8_t ModbusRegisterVariant::registerInput(uint16_t address, uint16_t &target){
    if(address >= wInput.size()) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    wInput[address].setAsPointer(&target);
    return 0;
}

uint8_t ModbusRegisterVariant::registerHold(uint16_t address, uint16_t *target){
    if(address >= wHold.size()) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    wHold[address].setAsPointer(target);
    return 0;
}
uint8_t ModbusRegisterVariant::registerHold(uint16_t address, uint16_t &target){
    if(address >= wHold.size()) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    wHold[address].setAsPointer(&target);
    return 0;
}

uint8_t ModbusRegisterVariant::setCoil(uint16_t address, uint8_t state){
    if(address >= bCoil.size()) return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    uint8_t tState = false;
    if(!bCoil[address].get(tState)) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    if(!bCoil[address].set(state)) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    uint16_t oldState = tState;
    if(onCoilSet) onCoilSet(this,address,oldState);
    return 0;
}

uint8_t ModbusRegisterVariant::getCoil(uint16_t address, uint8_t &state){
    if(address >= bCoil.size()) return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    uint16_t u16State = 0;
    if(onCoilGet && onCoilGet(this,address,u16State)){
        state = u16State;
    }else{
        if(!bCoil[address].get(state)) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    }
    return 0;
}

uint8_t ModbusRegisterVariant::getCoil(uint16_t address){
    uint8_t state = false;
    this->getCoil(address, state);
    return state;
}

uint8_t ModbusRegisterVariant::setDiscreteInput(uint16_t address, uint8_t state){
    if(address >= bDiscreteInput.size()) return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    uint8_t tState = false;
    if(!bDiscreteInput[address].get(tState)) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    if(!bDiscreteInput[address].set(state)) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    uint16_t oldState = tState;
    if(onDiscreteInputSet) onDiscreteInputSet(this,address,oldState);
    return 0;
}

uint8_t ModbusRegisterVariant::getDiscreteInput(uint16_t address, uint8_t &state){
    if(address >= bDiscreteInput.size()) return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    uint16_t u16State = 0;
    if(onDiscreteInputGet && onDiscreteInputGet(this,address,u16State)){
        state = u16State;
    }else{
        if(!bDiscreteInput[address].get(state)) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    }
    return 0;
}

uint8_t ModbusRegisterVariant::getDiscreteInput(uint16_t address){
    uint8_t state = false;
    this->getDiscreteInput(address, state);
    return state;
}

uint8_t ModbusRegisterVariant::setInput(uint16_t address, uint16_t data){
    if(address >= wInput.size()) return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    uint16_t oldState = 0;
    if(!wInput[address].get(oldState)) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    if(!wInput[address].set(data)) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    if(onInputSet) onInputSet(this,address,oldState);
    return 0;
}

uint16_t &ModbusRegisterVariant::getInputRef(uint16_t address){   //Only non-pointer register
    if(address >= wInput.size()) address = 0;
    return *(uint16_t*)&(wInput[address]);
}

uint8_t ModbusRegisterVariant::getInput(uint16_t address, uint16_t &data){
    if(address >= wInput.size()) return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    uint16_t u16State = 0;
    if(!(onInputGet && onInputGet(this,address,u16State)))
        if(!wInput[address].get(u16State))
            return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    data = u16State;
    return 0;
}

uint16_t ModbusRegisterVariant::getInput(uint16_t address){
    uint16_t state = 0;
    this->getInput(address, state);
    return state;
}

uint8_t ModbusRegisterVariant::setHold(uint16_t address, uint16_t data){
    if(address >= wHold.size()) return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    uint16_t oldState = 0;
    if(!wHold[address].get(oldState)) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    if(!wHold[address].set(data)) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    if(onHoldSet) onHoldSet(this,address,oldState);
    return 0;
}

inline void ModbusRegisterVariant::setHoldFast(uint16_t address, uint16_t data){
    if(address >= wHold.size()) return; //Out Of Range
    wHold[address].set(data);
}

uint16_t &ModbusRegisterVariant::getHoldRef(uint16_t address){   //Only non-pointer register
    if(address >= wHold.size()) address = 0;
    return *(uint16_t*)&(wHold[address]);
}

uint8_t ModbusRegisterVariant::getHold(uint16_t address, uint16_t &data){
    if(address >= wHold.size()) return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    uint16_t u16State = 0;
    if(!(onHoldGet && onHoldGet(this,address,u16State)))
        if(!wHold[address].get(u16State))
            return MBPDiagnose::DiagnoseCode_InvalidDataAddress;
    data = u16State;
    return 0;
}

uint16_t ModbusRegisterVariant::getHold(uint16_t address){
    uint16_t state = 0;
    this->getHold(address, state);
    return state;
}

inline void ModbusRegisterVariant::getHoldFast(uint16_t address, uint16_t &data){
    if(address >= wHold.size()) return; //Out Of Range
    wHold[address].get(data);
}

uint8_t ModbusRegisterVariant::process(ModbusFrame &frameIn, ModbusFrame &frameOut){
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
            uint8_t state = false;
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
            uint8_t state = false;
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
            pOut->setValue(i,result==0?state:0);
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
        #ifdef DEBUG_MODBUS_ON
        Serial.println("发送错误回包");
        #endif
        frameOut.createDiagnose(frameIn.pack->getFunctionCode());
        MBPDiagnose *pOutDiag = (MBPDiagnose *)(frameOut.pack);
        pOutDiag->setDiagnoseCode(result);
    }
    return result;
}

uint8_t ModbusRegisterVariant::processResponse(ModbusFrame &frameResponse, ModbusFrame &frameRequest){
    uint8_t result = 0;
    if(frameResponse.pack->getFunctionCode() != frameRequest.pack->getFunctionCode()) return 253;
    switch(frameResponse.pack->getFunctionCode()){
    case MBPReadCoilRegisterResponse::FunctionCode: {
        MBPReadCoilRegisterRequest *fReq = (MBPReadCoilRegisterRequest *)(frameRequest.pack);
        MBPReadCoilRegisterResponse *fResp = (MBPReadCoilRegisterResponse *)(frameResponse.pack);
        #ifdef DEBUG_MODBUS_ON
        Serial.println("读线圈");
        #endif
        uint16_t startAddress = fReq->getStartAddress();
        for(uint16_t i=0; i<fReq->getQuantity(); i++){
            result = this->setCoil(startAddress+i,fResp->getValue(i));
            if(result != 0) break;
        }
        break;
    }
    case MBPReadDiscreteInputRegisterResponse::FunctionCode: {
        MBPReadDiscreteInputRegisterRequest *fReq = (MBPReadDiscreteInputRegisterRequest *)(frameRequest.pack);
        MBPReadDiscreteInputRegisterResponse *fResp = (MBPReadDiscreteInputRegisterResponse *)(frameResponse.pack);
        #ifdef DEBUG_MODBUS_ON
        Serial.println("读离散输入");
        #endif
        uint16_t startAddress = fReq->getStartAddress();
        for(uint16_t i=0; i<fReq->getQuantity(); i++){
            result = this->setDiscreteInput(startAddress+i,fResp->getValue(i));
            if(result != 0) break;
        }
        break;
    }
    case MBPReadHoldingRegisterResponse::FunctionCode: {
        MBPReadHoldingRegisterRequest *fReq = (MBPReadHoldingRegisterRequest *)(frameRequest.pack);
        MBPReadHoldingRegisterResponse *fResp = (MBPReadHoldingRegisterResponse *)(frameResponse.pack);
        #ifdef DEBUG_MODBUS_ON
        Serial.println("读保持寄存器");
        #endif
        uint16_t startAddress = fReq->getStartAddress();
        for(uint16_t i=0; i<fReq->getQuantity(); i++){
            result = this->setHold(startAddress+i,fResp->getValue(i));
            if(result != 0) break;
        }
        break;
    }
    case MBPReadInputRegisterResponse::FunctionCode: {
        MBPReadInputRegisterRequest *fReq = (MBPReadInputRegisterRequest *)(frameRequest.pack);
        MBPReadInputRegisterResponse *fResp = (MBPReadInputRegisterResponse *)(frameResponse.pack);
        #ifdef DEBUG_MODBUS_ON
        Serial.println("读输入寄存器");
        #endif
        uint16_t startAddress = fReq->getStartAddress();
        for(uint16_t i=0; i<fReq->getQuantity(); i++){
            result = this->setInput(startAddress+i,fResp->getValue(i));
            if(result != 0) break;
        }
        break;
    }
    default:
        break; 
    }
    return result;
}





