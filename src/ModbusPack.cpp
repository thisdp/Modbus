#include "ModbusPack.h"

CRC16 gModbusCRC(CRC16MODBUS);

uint16_modbus::uint16_modbus(uint16_t value) {
    set(value);
}

/*******************************************Modbus帧*******************************************/
ModbusFrame::ModbusFrame(CRC16 *crcmgr) {
    crcMgr = crcmgr == 0?&gModbusCRC:crcmgr;
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
    if(pack->isDiagnosePack()) ((MBPDiagnose*)pack)->setDiagnoseCode(pBuffer[0]);
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
    crcMgr->clear();
    crc = (uint16_t*)(pack->endOfPack);
    crcMgr->update(station,(((int)crc)-((int)station)));
    return crcMgr->get() == *crc;
}

void ModbusFrame::applyCRC(){
    crcMgr->clear();
    crc = (uint16_t*)(pack->endOfPack);
    crcMgr->update(station,(((int)crc)-((int)station)));
    *crc = crcMgr->get();
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
void MBPReadCoilRegisterRequest::popRegisters(bool fromHead, uint16_t quant){
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
void MBPReadCoilRegisterResponse::pushRegisters(bool fromHead, uint16_t quant, uint8_t *data){
    //如果添加非16的整数倍的数量，则会扩展为16的整数倍对齐
    uint8_t deltaWords = (uint8_t)((quant+15)>>4);
    uint8_t origWords = getBytes()/2;
    uint8_t newWords = origWords+deltaWords;
    uint16_t *wValues = (uint16_t *)values;
    if(fromHead){  //从头部添加
        for (int8_t i = origWords - 1; i >= 0; i--) wValues[i + deltaWords] = wValues[i];
        for (uint8_t i = 0; i < deltaWords; i++) wValues[i] = ((uint16_t*)data)[i];
    }else{  //从末尾添加
        for (uint8_t i = 0; i < deltaWords; i++) wValues[i+origWords] = ((uint16_t*)data)[i];
    }
    *bytes = newWords*2;
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
void MBPReadDiscreteInputRegisterRequest::popRegisters(bool fromHead, uint16_t quant){
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
    /*Serial.println("Write");
    for(uint16_t i=0;i<*bytes;i++){
        Serial.println(values[i],BIN);
    }
    Serial.println("Done");*/
}
void MBPReadDiscreteInputRegisterResponse::pushRegisters(bool fromHead, uint16_t quant, uint8_t *data){
    _quantity = getBytes() * 16; //以16为整
    uint8_t deltaWords = (uint8_t)((quant+15)>>4);
    uint8_t origWords = getBytes()/2;
    uint8_t newWords = origWords+deltaWords;
    uint16_t *wValues = (uint16_t *)values;
    if(fromHead){  //从头部添加
        for (int8_t i = origWords - 1; i >= 0; i--) wValues[i + deltaWords] = wValues[i];
        for (uint8_t i = 0; i < deltaWords; i++) wValues[i] = ((uint16_t*)data)[i];
    }else{  //从末尾添加
        for (uint8_t i = 0; i < deltaWords; i++) wValues[i+origWords] = ((uint16_t*)data)[i];
    }
    *bytes = newWords*2;
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
void MBPReadHoldingRegisterRequest::popRegisters(bool fromHead, uint16_t quant){
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
void MBPReadHoldingRegisterResponse::pushRegisters(bool fromHead, uint16_t quant, uint8_t *data){
    _quantity = *bytes/2; //2字节
    uint8_t deltaBytes = (uint8_t)(quant*2);
    uint8_t origBytes = *bytes;
    uint8_t newBytes = origBytes+deltaBytes;
    uint8_t *bValues = (uint8_t*)values;
    if(fromHead){  //从头部添加
      for (uint8_t i = 0; i < origBytes; i++) bValues[i+deltaBytes] = bValues[i];
      for (uint8_t i = 0; i < deltaBytes; i++) bValues[i] = data[i];
    }else{  //从末尾添加
      for (uint8_t i = 0; i < deltaBytes; i++) bValues[i+origBytes] = data[i];
    }
    *bytes = newBytes;
    setEOP(bValues+getBytes());
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
void MBPReadInputRegisterRequest::popRegisters(bool fromHead, uint16_t quant){
    setQuantity(getQuantity()-quant);  //减去删除的寄存器数量
    /*if(fromHead){  //如果从头往前后
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
void MBPReadInputRegisterResponse::pushRegisters(bool fromHead, uint16_t quant, uint8_t *data){
    _quantity = *bytes/2; //2字节
    uint8_t deltaBytes = (uint8_t)(quant*2);
    uint8_t origBytes = *bytes;
    uint8_t newBytes = origBytes+deltaBytes;
    if(fromHead){  //从头部添加
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
    values = (uint16_t*)pBuffer;
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

void MBPWriteMultipleCoilRegistersRequest::pushRegisters(bool fromHead, uint16_t quant, uint8_t *data){
    //如果添加非8的整数倍的数量，则会扩展为8的整数倍对齐
    uint8_t deltaBytes = (uint8_t)((quant+15)>>4);
    uint8_t origBytes = *bytes;
    uint8_t newBytes = origBytes+deltaBytes;
    //从byteIndex位置插入
    for (uint8_t i = 0; i < origBytes; i++) values[i+deltaBytes] = values[i];
    for (uint8_t i = 0; i < deltaBytes; i++) values[i] = ((uint16_t*)data)[i];
    *bytes = newBytes;
    setQuantity(getQuantity()+quant);
    setEOP(((uint8_t*)values)+getBytes());
}
void MBPWriteMultipleCoilRegistersRequest::popRegisters(bool fromHead, uint16_t quant) {
    uint16_t totalQuantity = getQuantity();
    if(totalQuantity <= quant) quant = totalQuantity;
    uint16_t remainQuantity = totalQuantity-quant;
    uint8_t newValueCount = ((remainQuantity + 15) >> 4);
    if (fromHead) {  //如果从头往后删
        uint16_t fromWord = quant/16;
        uint16_t fromBit = quant%16;
        for (uint8_t i = 0; i < newValueCount; i++) {   //从0到newValueCount-1
            uint16_t val = values[i+fromWord];
            uint16_t valNext = 0;
            if(i+1 < newValueCount){
                valNext = values[i+1+fromWord];
            }
            val = val >> fromBit;
            val = val | ((valNext & (0xFFFF >> fromBit)) << (16-fromBit));
            values[i] = val;
        }
    }else{  //从末尾往前删
        uint16_t val = values[newValueCount - 1];
        uint8_t remainDecreaseQuant = remainQuantity % 16;  //清除剩余的位
        val = val & (0xFFFF >> (16-remainDecreaseQuant)); //移除高位数据
        values[newValueCount - 1] = val;
        // 不需要改变起始地址
    }
    *bytes = newValueCount*2;
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
void MBPWriteMultipleCoilRegistersResponse::pushRegisters(bool fromHead, uint16_t quant, uint8_t *data){
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
void MBPWriteMultipleHoldingRegistersRequest::popRegisters(bool fromHead, uint16_t quant){
    uint8_t deltaBytes = (uint8_t)(quant*2);
    uint8_t origBytes = *bytes;
    if(origBytes < deltaBytes){
        deltaBytes = origBytes;
        quant = deltaBytes*2;
    }
    uint8_t newBytes = origBytes-deltaBytes;
    uint8_t *u8Values = (uint8_t*)values;
    if(fromHead){  //从头部删除
        for (uint8_t i = 0; i < newBytes; i++) u8Values[i] = u8Values[i+deltaBytes];
    } //从末尾删除不需要移动数据
    *bytes = newBytes;
    setQuantity(getQuantity()-quant);
    setEOP(u8Values+getBytes());
}
void MBPWriteMultipleHoldingRegistersRequest::pushRegisters(bool fromHead, uint16_t quant, uint8_t *data){
    uint8_t deltaBytes = (uint8_t)(quant*2);
    uint8_t origBytes = *bytes;
    uint8_t newBytes = origBytes+deltaBytes;
    uint8_t *u8Values = (uint8_t*)values;
    if(fromHead){  //从头部添加
        for (uint8_t i = 0; i < origBytes; i++) u8Values[i+deltaBytes] = u8Values[i];
        for (uint8_t i = 0; i < deltaBytes; i++) u8Values[i] = data[i];
    }else{  //从末尾添加
        for (uint8_t i = 0; i < deltaBytes; i++) u8Values[i+origBytes] = data[i];
    }
    *bytes = newBytes;
    setQuantity(getQuantity()+quant);
    setEOP(u8Values+getBytes());
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
void MBPWriteMultipleHoldingRegistersResponse::pushRegisters(bool fromHead, uint16_t quant, uint8_t *data){
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
    default:{
        ModbusBasePack* mbPack = new MBPDiagnose();
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