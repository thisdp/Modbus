#include "ModbusPack.h"

CRC16 ModbusCRC(CRC16MODBUS);

uint16_modbus::uint16_modbus(uint16_t value) {
    set(value);
}

/*******************************************Modbus帧*******************************************/
ModbusFrame::ModbusFrame() {
    pack = 0;
    crc = 0;
    station = buffer;
    validDataLength = 0;
}

ModbusFrame::~ModbusFrame() {
    if (pack) delete pack;
}

void ModbusFrame::copy(ModbusFrame &frame, uint16_t length){
    memcpy(buffer,frame.buffer,length);
}

uint8_t* ModbusFrame::castRequest(bool isNew) {
    if (pack) {
        delete pack;
        pack = 0;
    }
    uint8_t *pBuffer = buffer;
    pBuffer += sizeof(uint8_t);
    pack = ModbusBasePack::CreateModbusRequestPack(pBuffer[0]);
    if(pack == 0) return 0;
    pBuffer = pack->cast(pBuffer,isNew);
    return pBuffer;
}

uint8_t* ModbusFrame::castResponse(bool isNew) {
    if (pack) {
        delete pack;
        pack = 0;
    }
    uint8_t *pBuffer = buffer;
    pBuffer += sizeof(uint8_t);
    pack = ModbusBasePack::CreateModbusResponsePack(pBuffer[0]);
    if(pack == 0) return 0;
    pBuffer = pack->cast(pBuffer,isNew);
    return pBuffer;
}

uint8_t* ModbusFrame::castDiagnose(bool isNew) {
    if (pack) {
        delete pack;
        pack = 0;
    }
    uint8_t *pBuffer = buffer;
    pBuffer += sizeof(uint8_t);
    pack = ModbusBasePack::CreateModbusDiagnosePack();
    if(pack == 0) return 0;
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

void ModbusFrame::writeRaw(Stream &s, uint16_t length){
    s.write(buffer, length);
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
//请求
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
void MBPReadCoilRegisterRequest::popRegisters(bool toTail, uint16_t quant){
    setQuantity(getQuantity()-quant);  //减去删除的寄存器数量
}
//回复
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
void MBPReadCoilRegisterResponse::pushRegisters(bool toTail, uint16_t quant, uint8_t *data){
    //如果添加非8的整数倍的数量，则会扩展为8的整数倍对齐
    _quantity = *bytes * 8; //以8为整
    uint8_t deltaBytes = (uint8_t)((quant+7)>>3);
    uint8_t origBytes = *bytes;
    uint8_t newBytes = origBytes+deltaBytes;
    if(!toTail){  //从头部添加
        for (uint8_t i = 0; i < origBytes; i++) values[i+deltaBytes] = values[i];
        for (uint8_t i = 0; i < deltaBytes; i++) values[i] = data[i];
    }else{  //从末尾添加
        for (uint8_t i = 0; i < deltaBytes; i++) values[i+origBytes] = data[i];
    }
    *bytes = newBytes;
    setEOP(((uint8_t*)values)+getBytes());
}

//读离散输入寄存器0x02
//请求
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
void MBPReadDiscreteInputRegisterRequest::popRegisters(bool toTail, uint16_t quant){
    setQuantity(getQuantity()-quant);  //减去删除的寄存器数量
}
//回复
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
void MBPReadDiscreteInputRegisterResponse::pushRegisters(bool toTail, uint16_t quant, uint8_t *data){
    _quantity = *bytes * 8; //以8为整
    uint8_t deltaBytes = (uint8_t)((quant+7)>>3);
    uint8_t origBytes = *bytes;
    uint8_t newBytes = origBytes+deltaBytes;
    if(!toTail){  //从头部添加
      for (uint8_t i = 0; i < origBytes; i++) values[i+deltaBytes] = values[i];
      for (uint8_t i = 0; i < deltaBytes; i++) values[i] = data[i];
    }else{  //从末尾添加
      for (uint8_t i = 0; i < deltaBytes; i++) values[i+origBytes] = data[i];
    }
    *bytes = newBytes;
    setEOP(((uint8_t*)values)+getBytes());
}
//读保持寄存器0x03
//请求
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
void MBPReadHoldingRegisterRequest::popRegisters(bool toTail, uint16_t quant){
    setQuantity(getQuantity()-quant);  //减去删除的寄存器数量
}
//回复
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
void MBPReadHoldingRegisterResponse::pushRegisters(bool toTail, uint16_t quant, uint8_t *data){
    _quantity = *bytes/2; //2字节
    uint8_t deltaBytes = (uint8_t)(quant*2);
    uint8_t origBytes = *bytes;
    uint8_t newBytes = origBytes+deltaBytes;
    if(!toTail){  //从头部添加
      for (uint8_t i = 0; i < origBytes; i++) values[i+deltaBytes] = values[i];
      for (uint8_t i = 0; i < deltaBytes; i++) values[i] = data[i];
    }else{  //从末尾添加
      for (uint8_t i = 0; i < deltaBytes; i++) values[i+origBytes] = data[i];
    }
    *bytes = newBytes;
    setEOP(((uint8_t*)values)+getBytes());
}
//读输入寄存器0x04
//请求
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
void MBPReadInputRegisterRequest::popRegisters(bool toTail, uint16_t quant){
    setQuantity(getQuantity()-quant);  //减去删除的寄存器数量
    /*if(!toTail){  //如果从头往前后
      if(!keepStartAddress){  //如果不保持起始地址
        setStartAddress(getStartAddress()+quant); //偏移起始地址
      }
    }*/
}
//回复
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
void MBPReadInputRegisterResponse::pushRegisters(bool toTail, uint16_t quant, uint8_t *data){
    _quantity = *bytes/2; //2字节
    uint8_t deltaBytes = (uint8_t)(quant*2);
    uint8_t origBytes = *bytes;
    uint8_t newBytes = origBytes+deltaBytes;
    if(!toTail){  //从头部添加
      for (uint8_t i = 0; i < origBytes; i++) values[i+deltaBytes] = values[i];
      for (uint8_t i = 0; i < deltaBytes; i++) values[i] = data[i];
    }else{  //从末尾添加
      for (uint8_t i = 0; i < deltaBytes; i++) values[i+origBytes] = data[i];
    }
    *bytes = newBytes;
    setEOP(((uint8_t*)values)+getBytes());
}
//写单个线圈寄存器0x05
//请求
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
//回复
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
//请求
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
//回复
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
//请求
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

void MBPWriteMultipleCoilRegistersRequest::pushRegisters(bool toTail, uint16_t quant, uint8_t *data){
    //如果添加非8的整数倍的数量，则会扩展为8的整数倍对齐
    uint8_t deltaBytes = (uint8_t)((quant+7)>>3);
    uint8_t origBytes = *bytes;
    uint8_t newBytes = origBytes+deltaBytes;
    //从byteIndex位置插入
    for (uint8_t i = 0; i < origBytes; i++) values[i+deltaBytes] = values[i];
    for (uint8_t i = 0; i < deltaBytes; i++) values[i] = data[i];
    *bytes = newBytes;
    setQuantity(getQuantity()+quant);
    setEOP(((uint8_t*)values)+getBytes());
}
void MBPWriteMultipleCoilRegistersRequest::popRegisters(bool toTail, uint16_t quant) {
    uint16_t totalQuantity = getQuantity();
    if(totalQuantity <= quant) quant = totalQuantity;
    uint16_t remainQuantity = totalQuantity-quant;
    uint8_t decreaseBytes = (uint8_t)(quant >> 3);
    uint8_t newBytes = ((remainQuantity + 7) >> 3);
    if (!toTail) {  //如果从头往后删
        uint8_t remainDecreaseQuant = quant - (decreaseBytes << 3);
        for (uint8_t i = 0; i < *bytes - decreaseBytes - 1; i++) {
            values[i] = (values[i + decreaseBytes] >> remainDecreaseQuant) | (values[i + decreaseBytes + 1] & (0xFF >> (8 - remainDecreaseQuant))) << (8 - remainDecreaseQuant);   //往前挪
        }
        values[*bytes - decreaseBytes - 1] = values[*bytes - decreaseBytes - 1] >> remainDecreaseQuant;
        /*if(!keepStartAddress){  //如果不保持起始地址
          setStartAddress(getStartAddress()+quant); //偏移起始地址
        }*/
    }else{  //从末尾往前删
        uint8_t remainDecreaseQuant = remainQuantity % 8;
        values[newBytes - 1] = values[newBytes - 1]&(0xFF >> (8-remainDecreaseQuant));    //移除高位数据
        // 不需要改变起始地址
    }
    *bytes = newBytes;
    setQuantity(remainQuantity);
    setEOP(((uint8_t*)values)+getBytes());
}
//回复
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
void MBPWriteMultipleCoilRegistersResponse::pushRegisters(bool toTail, uint16_t quant, uint8_t *data){
    setQuantity(getQuantity()+quant);
}

//写多个保持寄存器0x10
//请求
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
void MBPWriteMultipleHoldingRegistersRequest::popRegisters(bool toTail, uint16_t quant){
    uint8_t deltaBytes = (uint8_t)(quant*2);
    uint8_t origBytes = *bytes;
    if(origBytes < deltaBytes){
        deltaBytes = origBytes;
        quant = deltaBytes*2;
    }
    uint8_t newBytes = origBytes-deltaBytes;
    if(!toTail){  //从头部删除
        for (uint8_t i = 0; i < newBytes; i++) values[i] = values[i+deltaBytes];
    } //从末尾删除不需要移动数据
    *bytes = newBytes;
    setQuantity(getQuantity()-quant);
    setEOP(((uint8_t*)values)+getBytes());
}
void MBPWriteMultipleHoldingRegistersRequest::pushRegisters(bool toTail, uint16_t quant, uint8_t *data){
    uint8_t deltaBytes = (uint8_t)(quant*2);
    uint8_t origBytes = *bytes;
    uint8_t newBytes = origBytes+deltaBytes;
    if(!toTail){  //从头部添加
        for (uint8_t i = 0; i < origBytes; i++) values[i+deltaBytes] = values[i];
        for (uint8_t i = 0; i < deltaBytes; i++) values[i] = data[i];
    }else{  //从末尾添加
        for (uint8_t i = 0; i < deltaBytes; i++) values[i+origBytes] = data[i];
    }
    *bytes = newBytes;
    setQuantity(getQuantity()+quant);
    setEOP(((uint8_t*)values)+getBytes());
}
//回复
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
void MBPWriteMultipleHoldingRegistersResponse::pushRegisters(bool toTail, uint16_t quant, uint8_t *data){
    setQuantity(getQuantity()+quant);
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