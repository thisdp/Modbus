#include "ModbusPack.h"

CRC16 ModbusCRC(CRC16MODBUS);

uint16_modbus::uint16_modbus(uint16_t value) {
    set(value);
}

/*******************************************Modbus帧*******************************************/
ModbusFrame::ModbusFrame() {
    pack = 0;
    crc = 0;
    station = 0;
}

ModbusFrame::~ModbusFrame() {
    if (pack) delete pack;
}

uint8_t* ModbusFrame::castRequest(bool isNew) {
    if (pack) {
        delete pack;
        pack = 0;
    }
    uint8_t *pBuffer = buffer;
    station = pBuffer;
    pBuffer += sizeof(uint8_t);
    pack = ModbusBasePack::CreateModbusRequestPack(pBuffer[0]);
    pBuffer = pack->cast(pBuffer,isNew);
    return pBuffer;
}

uint8_t* ModbusFrame::castResponse(bool isNew) {
    if (pack) {
        delete pack;
        pack = 0;
    }
    uint8_t *pBuffer = buffer;
    station = pBuffer;
    pBuffer += sizeof(uint8_t);
    pack = ModbusBasePack::CreateModbusResponsePack(pBuffer[0]);
    pBuffer = pack->cast(pBuffer,isNew);
    return pBuffer;
}

uint8_t* ModbusFrame::castDiagnose(bool isNew) {
    if (pack) {
        delete pack;
        pack = 0;
    }
    uint8_t *pBuffer = buffer;
    station = pBuffer;
    pBuffer += sizeof(uint8_t);
    pack = ModbusBasePack::CreateModbusDiagnosePack();
    pBuffer = pack->cast(pBuffer,isNew);
    return pBuffer;
}

uint8_t* ModbusFrame::createRequest(uint8_t functionCode) {
    buffer[1] = functionCode;
    return castRequest(true);
}

uint8_t* ModbusFrame::createResponse(uint8_t functionCode) {
    buffer[1] = functionCode;
    return castResponse(true);
}

uint8_t* ModbusFrame::createDiagnose(uint8_t functionCode) {
    buffer[1] = functionCode|MBPDiagnose::FunctionCode;
    return castDiagnose(true);
}

bool ModbusFrame::verifyCRC(){
    ModbusCRC.clear();
    crc = (uint16_t*)(pack->endOfPack);
    ModbusCRC.update(station,(((int)crc)-((int)station)));
    return ModbusCRC.get() == *crc;
}

void ModbusFrame::applyCRC(){
    ModbusCRC.clear();
    crc = (uint16_t*)(pack->endOfPack);
    ModbusCRC.update(station,(((int)crc)-((int)station)));
    *crc = ModbusCRC.get();
}

void ModbusFrame::write(Stream &s){
    s.write((uint8_t*)station, 1);
    pack->write(s);
    s.write((uint8_t*)crc, 2);
}
/***************************数据包*****************************/
//基础包
void ModbusBasePack::write(Stream& s) {
    s.write(functionCode, 1);
}

uint8_t* ModbusBasePack::cast(uint8_t *pBuffer, bool isNew) {
    functionCode = pBuffer;
    pBuffer++;
    return pBuffer;
}

//诊断包
uint8_t* MBPDiagnose::cast(uint8_t *pBuffer, bool isNew) {
    pBuffer = ModbusBasePack::cast(pBuffer,isNew);
    diagnoseCode = pBuffer;
    pBuffer += sizeof(uint8_t);
    setEOP(pBuffer);
    return pBuffer;
}
void MBPDiagnose::write(Stream& s) {
    ModbusBasePack::write(s);
    s.write((uint8_t*)diagnoseCode, 1);
}

//读线圈寄存器0x01
uint8_t* MBPReadCoilRegisterRequest::cast(uint8_t *pBuffer, bool isNew) {
    pBuffer = ModbusBasePack::cast(pBuffer,isNew);
    startAddress = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    quantity = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    setEOP(pBuffer);
    if(isNew){  //Initialize Pack
        setStartAddress(0);
        setQuantity(0);
    }
    return pBuffer;
}
void MBPReadCoilRegisterRequest::write(Stream& s) {
    ModbusBasePack::write(s);
    s.write((uint8_t*)startAddress, 2);
    s.write((uint8_t*)quantity, 2);
}
uint8_t* MBPReadCoilRegisterResponse::cast(uint8_t *pBuffer, bool isNew) {
    pBuffer = ModbusBasePack::cast(pBuffer,isNew);
    bytes = pBuffer;
    pBuffer += sizeof(uint8_t);
    values = pBuffer;
    if(isNew){  //Initialize Pack
        initValues(0);
    }else{
        pBuffer += sizeof(uint8_t)*getBytes();
        setEOP(pBuffer);
    }
    return pBuffer;
}
void MBPReadCoilRegisterResponse::write(Stream& s) {
    ModbusBasePack::write(s);
    s.write((uint8_t*)bytes, 1);
    s.write((uint8_t*)values, *bytes);
}

//读离散输入寄存器0x02
uint8_t* MBPReadDiscreteInputRegisterRequest::cast(uint8_t *pBuffer, bool isNew) {
    pBuffer = ModbusBasePack::cast(pBuffer,isNew);
    startAddress = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    quantity = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    setEOP(pBuffer);
    if(isNew){  //Initialize Pack
        setStartAddress(0);
        setQuantity(0);
    }
    return pBuffer;
}
void MBPReadDiscreteInputRegisterRequest::write(Stream& s) {
    ModbusBasePack::write(s);
    s.write((uint8_t*)startAddress, 2);
    s.write((uint8_t*)quantity, 2);
}
uint8_t* MBPReadDiscreteInputRegisterResponse::cast(uint8_t *pBuffer, bool isNew) {
    pBuffer = ModbusBasePack::cast(pBuffer,isNew);
    bytes = pBuffer;
    pBuffer += sizeof(uint8_t);
    values = pBuffer;
    if(isNew){  //Initialize Pack
        initValues(0);
    }else{
        pBuffer += sizeof(uint8_t)*getBytes();
        setEOP(pBuffer);
    }
    return pBuffer;
}
void MBPReadDiscreteInputRegisterResponse::write(Stream& s) {
    ModbusBasePack::write(s);
    s.write((uint8_t*)bytes, 1);
    s.write((uint8_t*)values, *bytes);
}

//读保持寄存器0x03
uint8_t* MBPReadHoldingRegisterRequest::cast(uint8_t *pBuffer, bool isNew) {
    pBuffer = ModbusBasePack::cast(pBuffer,isNew);
    startAddress = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    quantity = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    setEOP(pBuffer);
    if(isNew){  //Initialize Pack
        setStartAddress(0);
        setQuantity(0);
    }
    return pBuffer;
}
void MBPReadHoldingRegisterRequest::write(Stream& s) {
    ModbusBasePack::write(s);
    s.write((uint8_t*)startAddress, 2);
    s.write((uint8_t*)quantity, 2);
}
uint8_t* MBPReadHoldingRegisterResponse::cast(uint8_t *pBuffer, bool isNew) {
    pBuffer = ModbusBasePack::cast(pBuffer,isNew);
    bytes = pBuffer;
    pBuffer += sizeof(uint8_t);
    values = (uint16_modbus*)pBuffer;
    if(isNew){  //Initialize Pack
        initValues(0);
    }else{
        pBuffer += sizeof(uint8_t)*getBytes();
        setEOP(pBuffer);
    }
    return pBuffer;
}
void MBPReadHoldingRegisterResponse::write(Stream& s) {
    ModbusBasePack::write(s);
    s.write((uint8_t*)bytes, 1);
    s.write((uint8_t*)values, *bytes);
}

//读输入寄存器0x04
uint8_t* MBPReadInputRegisterRequest::cast(uint8_t *pBuffer, bool isNew) {
    pBuffer = ModbusBasePack::cast(pBuffer,isNew);
    startAddress = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    quantity = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    setEOP(pBuffer);
    if(isNew){  //Initialize Pack
        setStartAddress(0);
        setQuantity(0);
    }
    return pBuffer;
}
void MBPReadInputRegisterRequest::write(Stream& s) {
    ModbusBasePack::write(s);
    s.write((uint8_t*)startAddress, 2);
    s.write((uint8_t*)quantity, 2);
}
uint8_t* MBPReadInputRegisterResponse::cast(uint8_t *pBuffer, bool isNew) {
    pBuffer = ModbusBasePack::cast(pBuffer,isNew);
    bytes = pBuffer;
    pBuffer += sizeof(uint8_t);
    values = (uint16_modbus*)pBuffer;
    if(isNew){  //Initialize Pack
        initValues(0);
    }else{
        pBuffer += sizeof(uint8_t)*getBytes();
        setEOP(pBuffer);
    }
    return pBuffer;
}
void MBPReadInputRegisterResponse::write(Stream& s) {
    ModbusBasePack::write(s);
    s.write((uint8_t*)bytes, 1);
    s.write((uint8_t*)values, *bytes);
}

//写单个线圈寄存器0x05
uint8_t* MBPWriteCoilRegisterRequest::cast(uint8_t *pBuffer, bool isNew) {
    pBuffer = ModbusBasePack::cast(pBuffer,isNew);
    startAddress = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    value = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    setEOP(pBuffer);
    if(isNew){  //Initialize Pack
        setStartAddress(0);
        setValue(0);
    }
    return pBuffer;
}
void MBPWriteCoilRegisterRequest::write(Stream& s) { //回复和请求报文一样
    ModbusBasePack::write(s);
    s.write((uint8_t*)startAddress, 2);
    s.write((uint8_t*)value, 2);
}
uint8_t* MBPWriteCoilRegisterResponse::cast(uint8_t *pBuffer, bool isNew) {
    pBuffer = ModbusBasePack::cast(pBuffer,isNew);
    startAddress = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    value = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    setEOP(pBuffer);
    if(isNew){  //Initialize Pack
        setStartAddress(0);
        setValue(0);
    }
    return pBuffer;
}
void MBPWriteCoilRegisterResponse::write(Stream& s) { //回复和请求报文一样
    ModbusBasePack::write(s);
    s.write((uint8_t*)startAddress, 2);
    s.write((uint8_t*)value, 2);
}

//写单个保持寄存器0x06
uint8_t* MBPWriteHoldingRegisterRequest::cast(uint8_t *pBuffer, bool isNew) {
    pBuffer = ModbusBasePack::cast(pBuffer,isNew);
    startAddress = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    value = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    setEOP(pBuffer);
    if(isNew){  //Initialize Pack
        setStartAddress(0);
        setValue(0);
    }
    return pBuffer;
}
void MBPWriteHoldingRegisterRequest::write(Stream& s) { //回复和请求报文一样
    ModbusBasePack::write(s);
    s.write((uint8_t*)startAddress, 2);
    s.write((uint8_t*)value, 2);
}
uint8_t* MBPWriteHoldingRegisterResponse::cast(uint8_t *pBuffer, bool isNew) {
    pBuffer = ModbusBasePack::cast(pBuffer,isNew);
    startAddress = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    value = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    setEOP(pBuffer);
    if(isNew){  //Initialize Pack
        setStartAddress(0);
        setValue(0);
    }
    return pBuffer;
}
void MBPWriteHoldingRegisterResponse::write(Stream& s) { //回复和请求报文一样
    ModbusBasePack::write(s);
    s.write((uint8_t*)startAddress, 2);
    s.write((uint8_t*)value, 2);
}

//写多个线圈寄存器0x0F
uint8_t* MBPWriteMultipleCoilRegistersRequest::cast(uint8_t *pBuffer, bool isNew) {
    pBuffer = ModbusBasePack::cast(pBuffer,isNew);
    startAddress = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    quantity = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    bytes = pBuffer;
    pBuffer += sizeof(uint8_t);
    values = pBuffer;
    if(isNew){  //Initialize Pack
        setStartAddress(0);
        initValues(0);
    }else{
        pBuffer += sizeof(uint8_t)*getBytes();
        setEOP(pBuffer);
    }
    return pBuffer;
}
void MBPWriteMultipleCoilRegistersRequest::write(Stream& s) {
    ModbusBasePack::write(s);
    s.write((uint8_t*)startAddress, 2);
    s.write((uint8_t*)quantity, 2);
    s.write((uint8_t*)bytes, 1);
    s.write((uint8_t*)values, *bytes);
}
uint8_t* MBPWriteMultipleCoilRegistersResponse::cast(uint8_t *pBuffer, bool isNew) {
    pBuffer = ModbusBasePack::cast(pBuffer,isNew);
    startAddress = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    quantity = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    setEOP(pBuffer);
    if(isNew){  //Initialize Pack
        setStartAddress(0);
        setQuantity(0);
    }
    return pBuffer;
}
void MBPWriteMultipleCoilRegistersResponse::write(Stream& s) {
    ModbusBasePack::write(s);
    s.write((uint8_t*)startAddress, 2);
    s.write((uint8_t*)quantity, 2);
}

//写多个保持寄存器0x10
uint8_t* MBPWriteMultipleHoldingRegistersRequest::cast(uint8_t *pBuffer, bool isNew) {
    pBuffer = ModbusBasePack::cast(pBuffer,isNew);
    startAddress = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    quantity = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    bytes = pBuffer;
    pBuffer += sizeof(uint8_t);
    values = (uint16_modbus*)pBuffer;
    if(isNew){  //Initialize Pack
        setStartAddress(0);
        initValues(0);
    }else{
        pBuffer += sizeof(uint8_t)*getBytes();
        setEOP(pBuffer);
    }
    return pBuffer;
}
void MBPWriteMultipleHoldingRegistersRequest::write(Stream& s) {
    ModbusBasePack::write(s);
    s.write((uint8_t*)startAddress, 2);
    s.write((uint8_t*)quantity, 2);
    s.write((uint8_t*)bytes, 1);
    s.write((uint8_t*)values, *bytes);
}
uint8_t* MBPWriteMultipleHoldingRegistersResponse::cast(uint8_t *pBuffer, bool isNew) {
    pBuffer = ModbusBasePack::cast(pBuffer,isNew);
    startAddress = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    quantity = (uint16_modbus*)(pBuffer);
    pBuffer += sizeof(uint16_modbus);
    setEOP(pBuffer);
    if(isNew){  //Initialize Pack
        setStartAddress(0);
        setQuantity(0);
    }
    return pBuffer;
}
void MBPWriteMultipleHoldingRegistersResponse::write(Stream& s) {
    ModbusBasePack::write(s);
    s.write((uint8_t*)startAddress, 2);
    s.write((uint8_t*)quantity, 2);
}

ModbusBasePack *ModbusBasePack::CreateModbusDiagnosePack(){
    ModbusBasePack *mbPack = new MBPDiagnose();
    return mbPack;
}

ModbusBasePack *ModbusBasePack::CreateModbusRequestPack(uint8_t functionCode) {
    switch (functionCode) {   //功能码
    case MBPReadCoilRegisterRequest::FunctionCode: {
        ModbusBasePack *mbPack = new MBPReadCoilRegisterRequest();
        return mbPack;
    }
    case MBPReadDiscreteInputRegisterRequest::FunctionCode: {
        ModbusBasePack* mbPack = new MBPReadDiscreteInputRegisterRequest();
        return mbPack;
    }
    case MBPReadHoldingRegisterRequest::FunctionCode: {
        ModbusBasePack* mbPack = new MBPReadHoldingRegisterRequest();
        return mbPack;
    }
    case MBPReadInputRegisterRequest::FunctionCode: {
        ModbusBasePack* mbPack = new MBPReadInputRegisterRequest();
        return mbPack;
    }
    case MBPWriteCoilRegisterRequest::FunctionCode: {
        ModbusBasePack* mbPack = new MBPWriteCoilRegisterRequest();
        return mbPack;
    }
    case MBPWriteHoldingRegisterRequest::FunctionCode: {
        ModbusBasePack* mbPack = new MBPWriteHoldingRegisterRequest();
        return mbPack;
    }
    case MBPWriteMultipleCoilRegistersRequest::FunctionCode: {
        ModbusBasePack* mbPack = new MBPWriteMultipleCoilRegistersRequest();
        return mbPack;
    }
    case MBPWriteMultipleHoldingRegistersRequest::FunctionCode: {
        ModbusBasePack* mbPack = new MBPWriteMultipleHoldingRegistersRequest();
        return mbPack;
    }
    }
    return 0;
}

ModbusBasePack *ModbusBasePack::CreateModbusResponsePack(uint8_t functionCode) {
    switch (functionCode) {   //功能码
    case MBPReadCoilRegisterResponse::FunctionCode: {
        ModbusBasePack* mbPack = new MBPReadCoilRegisterResponse();
        return mbPack;
    }
    case MBPReadDiscreteInputRegisterResponse::FunctionCode: {
        ModbusBasePack* mbPack = new MBPReadDiscreteInputRegisterResponse();
        return mbPack;
    }
    case MBPReadHoldingRegisterResponse::FunctionCode: {
        ModbusBasePack* mbPack = new MBPReadHoldingRegisterResponse();
        return mbPack;
    }
    case MBPReadInputRegisterResponse::FunctionCode: {
        ModbusBasePack* mbPack = new MBPReadInputRegisterResponse();
        return mbPack;
    }
    case MBPWriteCoilRegisterResponse::FunctionCode: {
        ModbusBasePack* mbPack = new MBPWriteCoilRegisterResponse();
        return mbPack;
    }
    case MBPWriteHoldingRegisterResponse::FunctionCode: {
        ModbusBasePack* mbPack = new MBPWriteHoldingRegisterResponse();
        return mbPack;
    }
    case MBPWriteMultipleCoilRegistersResponse::FunctionCode: {
        ModbusBasePack* mbPack = new MBPWriteMultipleCoilRegistersResponse();
        return mbPack;
    }
    case MBPWriteMultipleHoldingRegistersResponse::FunctionCode: {
        ModbusBasePack* mbPack = new MBPWriteMultipleHoldingRegistersResponse();
        return mbPack;
    }
    }
    return 0;
}
/*
ModbusBasePack* ModbusBasePack::CastModbusRequestPack(uint8_t* p) {
    ModbusBasePack pack = CreateModbusRequestPack(p[0]);
    pack.cast(p);
    return pack;
}


ModbusBasePack ModbusBasePack::CastModbusResponsePack(uint8_t* p) {
    ModbusBasePack pack = CreateModbusResponsePack(p[0]);
    pack.cast(p);
    return pack;
}
*/