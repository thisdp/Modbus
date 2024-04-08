#include "ModbusRegister.h"

ModbusRegister::ModbusRegister(ModbusRegisterConfig regConfig) {
    regConf = regConfig;
    onHoldSet = 0;
    onCoilSet = 0;
    pbCoil = 0;
    pbDiscreteInput = 0;
    pwInput = 0;
    pwHold = 0;
    bCoil = 0;
    bDiscreteInput = 0;
    wInput = 0;
    wHold = 0;
    if (regConf.bCoilCount != 0){
        bCoil = new uint8_t[(regConf.bCoilCount / 8) + 1];
        for(uint8_t i=0;i<((regConf.bCoilCount+7) / 8);i++) bCoil[i] = 0;
    }
    if (regConf.bDiscreteInputCount != 0){
        bDiscreteInput = new uint8_t[(regConf.bDiscreteInputCount / 8) + 1];
        for(uint8_t i=0;i<((regConf.bDiscreteInputCount+7) / 8);i++) bDiscreteInput[i] = 0;
    }
    if (regConf.wInputCount != 0){
        wInput = new uint16_t[regConf.wInputCount];
        for(uint8_t i=0;i<regConf.wInputCount;i++) wInput[i] = 0;
    }
    if (regConf.wHoldCount != 0){
        wHold = new uint16_t[regConf.wHoldCount];
        for(uint8_t i=0;i<regConf.wHoldCount;i++) wHold[i] = 0;
    }

    
    if (regConf.pbCoilCount != 0){
        pbCoil = new uint8_t*[(regConf.pbCoilCount / 8) + 1];
        for(uint8_t i=0;i<((regConf.pbCoilCount+7) / 8);i++) pbCoil[i] = 0;
    }
    if (regConf.pbDiscreteInputCount != 0){
        pbDiscreteInput = new uint8_t*[(regConf.pbDiscreteInputCount / 8) + 1];
        for(uint8_t i=0;i<((regConf.pbDiscreteInputCount+7) / 8);i++) pbDiscreteInput[i] = 0;
    }
    if (regConf.pwInputCount != 0){
        pwInput = new uint16_t*[regConf.pwInputCount];
        for(uint8_t i=0;i<regConf.pwInputCount;i++) pwInput[i] = 0;
    }
    if (regConf.pwHoldCount != 0){
        pwHold = new uint16_t*[regConf.pwHoldCount];
        for(uint8_t i=0;i<regConf.pwHoldCount;i++) pwHold[i] = 0;
    }

}

ModbusRegister::~ModbusRegister() {
    if(bCoil) delete bCoil;
    if (bDiscreteInput) delete bDiscreteInput;
    if (wInput) delete wInput;
    if (wHold) delete wHold;
}

void ModbusRegister::registerCoil(uint16_t address, uint8_t *memAddress){   //一次必须映射8个线圈
    if(address < regConf.pbCoilCount) pbCoil[address%8] = memAddress;
}

void ModbusRegister::registerDiscreteInput(uint16_t address, uint8_t *memAddress){  //一次必须映射8个输入状态
    if(address < regConf.pbDiscreteInputCount) pbDiscreteInput[address%8] = memAddress;
}

void ModbusRegister::registerInput(uint16_t address, uint16_t *memAddress){
    if(address < regConf.pwInputCount) pwInput[address] = memAddress;
}

void ModbusRegister::registerHold(uint16_t address, uint16_t *memAddress){
    if(address < regConf.pwHoldCount) pwHold[address] = memAddress;
}

uint8_t ModbusRegister::setCoil(uint16_t address, bool state){
    uint16_t bitBlock = address/8;
    uint8_t bitIndex = address%8;
    if(address < regConf.pbCoilCount){
        if(pbCoil == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Coil Register Pointer
        *(pbCoil[bitBlock]) &= ~(1 << bitIndex);
        *(pbCoil[bitBlock]) |= (state << bitIndex);
        if(onCoilSet) onCoilSet(this,address);
    }else if(address < regConf.pbCoilCount+regConf.bCoilCount){
        if(bCoil == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Coil Register
        bCoil[bitBlock] &= ~(1 << bitIndex);
        bCoil[bitBlock] |= (state << bitIndex);
        if(onCoilSet) onCoilSet(this,address);
    }else{
        return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    }
    return 0;
}

uint8_t ModbusRegister::getCoil(uint16_t address, bool &state){
    uint16_t bitBlock = address/8;
    uint8_t bitIndex = address%8;
    if(address < regConf.pbCoilCount){
        if(pbCoil == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Coil Register Pointer
        state = (*(pbCoil[bitBlock]) >> bitIndex)&0x01;
    }else if(address < regConf.pbCoilCount+regConf.bCoilCount){
        if(bCoil == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Coil Register
        state = (bCoil[bitBlock] >> bitIndex)&0x01;
    }else{
        return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    }
    return 0;
}

bool ModbusRegister::getCoil(uint16_t address){
    bool state = false;
    getCoil(address, state);
    return state;
}

uint8_t ModbusRegister::getDiscreteInput(uint16_t address, bool &state){
    uint16_t bitBlock = address/8;
    uint8_t bitIndex = address%8;
    if(address < regConf.pbDiscreteInputCount){
        if(pbDiscreteInput == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Discrete Input Register Pointer
        state = (*(pbDiscreteInput[bitBlock]) >> bitIndex)&0x01;
    }else if(address < regConf.pbDiscreteInputCount+regConf.bDiscreteInputCount){
        if(bDiscreteInput == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Discrete Input Register
        state = (bDiscreteInput[bitBlock] >> bitIndex)&0x01;
    }else{
        return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    }
    return 0;
}

bool ModbusRegister::getDiscreteInput(uint16_t address){
    bool state = false;
    getDiscreteInput(address, state);
    return state;
}

uint8_t ModbusRegister::getInput(uint16_t address, uint16_t &data){
    if(address < regConf.pwInputCount){
        if(pwInput == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Input Register Pointer
        data = *(pwInput[address]);
    }else if(address < regConf.pwInputCount+regConf.wInputCount){
        if(wInput == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Input Register
        data = wInput[address];
    }else{
        return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    }
    return 0;
}

uint16_t ModbusRegister::getInput(uint16_t address){
    uint16_t state = 0;
    getInput(address, state);
    return state;
}

uint8_t ModbusRegister::setHold(uint16_t address, uint16_t data){
    if(address < regConf.pwHoldCount){
        if(pwHold == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Input Register Pointer
        *(pwHold[address]) = data;
        if(onHoldSet) onHoldSet(this,address);
    }else if(address < regConf.pwHoldCount+regConf.wHoldCount){
        if(wHold == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Input Register
        wHold[address] = data;
        if(onHoldSet) onHoldSet(this,address);
    }else{
        return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    }
    return 0;
}

uint8_t ModbusRegister::getHold(uint16_t address, uint16_t &data){
    if(address < regConf.pwHoldCount){
        if(pwHold == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Input Register Pointer
        data = *(pwHold[address]);
    }else if(address < regConf.pwHoldCount+regConf.wHoldCount){
        if(wHold == 0) return MBPDiagnose::DiagnoseCode_InvalidDataAddress;   //No Input Register
        data = wHold[address];
    }else{
        return MBPDiagnose::DiagnoseCode_InvalidDataAddress; //Out Of Range
    }
    return 0;
}

uint16_t ModbusRegister::getHold(uint16_t address){
    uint16_t state = 0;
    getHold(address, state);
    return state;
}

uint8_t ModbusRegister::process(ModbusFrame &frameIn, ModbusFrame &frameOut){
    uint8_t result = 0;
    frameOut.createResponse(frameIn.pack->getFunctionCode());
    switch(frameIn.pack->getFunctionCode()){
    case MBPReadCoilRegisterRequest::FunctionCode: {
        MBPReadCoilRegisterRequest *pIn = (MBPReadCoilRegisterRequest *)(frameIn.pack);
        MBPReadCoilRegisterResponse *pOut = (MBPReadCoilRegisterResponse *)(frameOut.pack);
        Serial.println("读线圈");
        pOut->initValues(pIn->getQuantity());
        uint16_t startAddress = pIn->getStartAddress();
        for(uint16_t i=0; i<pIn->getQuantity(); i++){
            bool state = false;
            result = getCoil(startAddress+i,state);
            if(result != 0) break;
            pOut->setValue(i,state);
        }
        break;
    }
    case MBPReadDiscreteInputRegisterRequest::FunctionCode: {
        MBPReadDiscreteInputRegisterRequest *pIn = (MBPReadDiscreteInputRegisterRequest *)(frameIn.pack);
        MBPReadDiscreteInputRegisterResponse *pOut = (MBPReadDiscreteInputRegisterResponse *)(frameOut.pack);
        Serial.println("读离散输入");
        pOut->initValues(pIn->getQuantity());
        uint16_t startAddress = pIn->getStartAddress();
        for(uint16_t i=0; i<pIn->getQuantity(); i++){
            bool state = false;
            result = getDiscreteInput(startAddress+i,state);
            if(result != 0) break;
            pOut->setValue(i,state);
        }
        break;
    }
    case MBPReadHoldingRegisterRequest::FunctionCode: {
        MBPReadHoldingRegisterRequest *pIn = (MBPReadHoldingRegisterRequest *)(frameIn.pack);
        MBPReadHoldingRegisterResponse *pOut = (MBPReadHoldingRegisterResponse *)(frameOut.pack);
        Serial.println("读保持寄存器");
        pOut->initValues(pIn->getQuantity());
        uint16_t startAddress = pIn->getStartAddress();
        for(uint16_t i=0; i<pIn->getQuantity(); i++){
            uint16_t state = false;
            result = getHold(startAddress+i,state);
            if(result != 0) break;
            pOut->setValue(i,state);
        }
        break;
    }
    case MBPReadInputRegisterRequest::FunctionCode: {
        MBPReadInputRegisterRequest *pIn = (MBPReadInputRegisterRequest *)(frameIn.pack);
        MBPReadInputRegisterResponse *pOut = (MBPReadInputRegisterResponse *)(frameOut.pack);
        Serial.println("读输入寄存器");
        pOut->initValues(pIn->getQuantity());
        uint16_t startAddress = pIn->getStartAddress();
        for(uint16_t i=0; i<pIn->getQuantity(); i++){
            uint16_t state = false;
            result = getInput(startAddress+i,state);
            if(result != 0) break;
            pOut->setValue(i,state);
        }
        break;
    }
    case MBPWriteCoilRegisterRequest::FunctionCode: {
        MBPWriteCoilRegisterRequest *pIn = (MBPWriteCoilRegisterRequest *)(frameIn.pack);
        MBPWriteCoilRegisterResponse *pOut = (MBPWriteCoilRegisterResponse *)(frameOut.pack);
        uint16_t startAddress = pIn->getStartAddress();
        Serial.println("写单线圈开始");
        Serial.print(startAddress);
        Serial.print(":");
        Serial.println(pIn->getValue());
        Serial.println(pIn->value->get());
        Serial.println("写单线圈结束");
        result = setCoil(startAddress,pIn->getValue());
        if(result != 0) break;
        pOut->setValue(pIn->getValue());
        pOut->setStartAddress(pIn->getStartAddress());
        break;
    }
    case MBPWriteHoldingRegisterRequest::FunctionCode: {
        MBPWriteHoldingRegisterRequest *pIn = (MBPWriteHoldingRegisterRequest *)(frameIn.pack);
        MBPWriteHoldingRegisterResponse *pOut = (MBPWriteHoldingRegisterResponse *)(frameOut.pack);
        uint16_t startAddress = pIn->getStartAddress();
        Serial.println("写单保持寄存器开始");
        Serial.print(startAddress);
        Serial.print(":");
        Serial.println(pIn->getValue());
        Serial.println("写单保持寄存器结束");
        result = setHold(startAddress,pIn->getValue());
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
        Serial.println("写多线圈开始");
        for(uint16_t i=0; i<pIn->getQuantity(); i++){
            result = setCoil(startAddress+i,pIn->getValue(i));
            Serial.print(startAddress+i);
            Serial.print(":");
            Serial.println(pIn->getValue(i));
            if(result != 0) break;
        }
        Serial.println("写多线圈结束");
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
        Serial.println("写多保持寄存器开始");
        for(uint16_t i=0; i<pIn->getQuantity(); i++){
            result = setHold(startAddress+i,pIn->getValue(i));
            Serial.print(startAddress+i);
            Serial.print(":");
            Serial.println(pIn->getValue(i));
            if(result != 0) break;
        }
        Serial.println("写多保持寄存器结束");
        pOut->setStartAddress(startAddress);
        pOut->setQuantity(pIn->getQuantity());
        break;
    } 
    }
    if(result != 0){
        Serial.println("发送错误回包");
        frameOut.createDiagnose(frameIn.pack->getFunctionCode());
        MBPDiagnose *pOutDiag = (MBPDiagnose *)(frameOut.pack);
        pOutDiag->setDiagnoseCode(result);
        Serial.println("设置完毕");
    }
    return result;
}