#include "LIS2MDL_Driver.h"

#ifdef DEBUG
#define DEBUG_PRINTER Serial
#define DEBUG_PRINT(...)                  \
    {                                     \
        DEBUG_PRINTER.print(__VA_ARGS__); \
    }
#define DEBUG_PRINTLN(...)                  \
    {                                       \
        DEBUG_PRINTER.println(__VA_ARGS__); \
    }
#define DEBUG_PRINTF(...)                  \
    {                                      \
        DEBUG_PRINTER.printf(__VA_ARGS__); \
    }
#define DEBUG_BEGIN(...)                  \
    {                                     \
        DEBUG_PRINTER.begin(__VA_ARGS__); \
    }
#else
#define DEBUG_PRINT(...) \
    {                    \
    }
#define DEBUG_PRINTLN(...) \
    {                      \
    }
#define DEBUG_PRINTF(...) \
    {                     \
    }
#define DEBUG_BEGIN(...) \
    {                    \
    }
#endif

bool everyXms(long &timer, long duration)
{
    if (millis() - timer > duration)
    {
        timer = millis();
        return true;
    }
    return false;
}

LIS2MDL::LIS2MDL()
{
}

bool LIS2MDL::begin()
{
    Wire.beginTransmission(COMPASS_DEVICEADDRESS);
    if (Wire.endTransmission(true) != 0)
    {
        return false;
    }

    Config_A_Type AConfig;
    uint8_t AConfigraw = DEFAULT_CONFIG_A;
    memcpy(&AConfig, &AConfigraw, 1);
    writeConfigA(AConfig);

    Config_B_Type BConfig;
    uint8_t BConfigraw = DEFAULT_CONFIG_B;
    memcpy(&BConfig, &BConfigraw, 1);
    setConfigB(BConfig);

    Config_C_Type Config;
    uint8_t CConfigraw = DEFAULT_CONFIG_C;
    memcpy(&Config, &CConfigraw, 1);
    setConfigC(Config);

    enableOffsetCancelation();

    return true;
}

void LIS2MDL::readRaw(int16_t &X, int16_t &Y, int16_t &Z)
{
    Status_type status;
    status = readStatus();
    if (status.XYZ_available)
    {
        X = readXRaw();
        Y = readYRaw();
        Z = readZRaw();
    }
}

void LIS2MDL::readMag(float &X, float &Y, float &Z)
{
    Status_type status;
    status = readStatus();
    if (status.XYZ_available)
    {
        X = (float)readXRaw() * LSB_TO_MSG;
        Y = (float)readYRaw() * LSB_TO_MSG;
        Z = (float)readZRaw() * LSB_TO_MSG;
    }
}

void LIS2MDL::calibrate()
{
    long startTime = millis();
    int16_t minX = 1000;
    int16_t maxX = -1000;
    int16_t minY = 1000;
    int16_t maxY = -1000;
    int16_t X, Y, Z;
    enableOffsetCancelation(false);
    writeXOffset(0);
    writeYOffset(0);
    long printTimer = 0;

    // throw away some readings, the first ones seems to be erroneous
    for (int i = 0; i < 10; i++)
    {
        readRaw(X, Y, Z);
        delay(100);
    }
    while (millis() - startTime < 30000)
    {
        readRaw(X, Y, Z);
        if (X < minX)
            minX = X;
        if (X > maxX)
            maxX = X;
        if (Y < minY)
            minY = Y;
        if (Y > maxY)
            maxY = Y;
        delay(10);
        if (everyXms(printTimer, 1000))
        {
            printTimer = millis();
            Serial.print("X: ");
            Serial.print(X);
            Serial.print(" Y: ");
            Serial.println(Y);
        }
    }

    writeXOffset((int16_t)((minX + maxX) / 2));
    writeYOffset((int16_t)((minY + maxY) / 2));

    enableOffsetCancelation();

    Serial.print("X: ");
    Serial.print(minX);
    Serial.print(" : ");
    Serial.println(maxX);
    Serial.print("Y: ");
    Serial.print(minY);
    Serial.print(" : ");
    Serial.println(maxY);
    Serial.println();
}

float LIS2MDL::getHeading()
{

    readMag(X, Y, Z);

    float Pi = 3.14159;

    // Calculate the angle of the vector y,x
    float heading = (atan2(Y, X) * 180) / Pi;

    // Normalize to 0-360
    if (heading < 0)
    {
        heading = 360 + heading;
    }
    return heading;
}

void LIS2MDL::writeXOffset(int16_t setVal)
{
    writeByte(OFFSET_X_REG_H_ADDR, setVal >> 8);
    writeByte(OFFSET_X_REG_L_ADDR, setVal & 0xFFFF);
}

int16_t LIS2MDL::readXOffset()
{
    int8_t upper = readByte(OFFSET_X_REG_H_ADDR);
    int8_t lower = readByte(OFFSET_X_REG_L_ADDR);
    return (upper << 8) | lower;
}

void LIS2MDL::writeYOffset(int16_t setVal)
{
    writeByte(OFFSET_Y_REG_H_ADDR, setVal >> 8);
    writeByte(OFFSET_Y_REG_L_ADDR, setVal & 0xFFFF);
}

int16_t LIS2MDL::readYOffset()
{
    uint8_t upper = readByte(OFFSET_Y_REG_H_ADDR);
    uint8_t lower = readByte(OFFSET_Y_REG_L_ADDR);
    return (upper << 8) | lower;
}

void LIS2MDL::writeZOffset(int16_t setVal)
{
    writeByte(OFFSET_Z_REG_H_ADDR, setVal >> 8);
    writeByte(OFFSET_Z_REG_L_ADDR, setVal & 0xFFFF);
}

int16_t LIS2MDL::readZOffset()
{
    uint8_t upper = readByte(OFFSET_Z_REG_H_ADDR);
    uint8_t lower = readByte(OFFSET_Z_REG_L_ADDR);
    return (upper << 8) | lower;
}

// CFG_REG_A
Config_A_Type LIS2MDL::readConfigA()
{
    Config_A_Type config;
    uint8_t data = readByte(CFG_REG_A_ADDR);
    memcpy(&config, &data, sizeof(data));
    return config;
}

void LIS2MDL::writeConfigA(Config_A_Type config)
{
    writeByte(CFG_REG_A_ADDR, (uint8_t *)&config);
}

void LIS2MDL::enableTempComp(bool enable)
{
    Config_A_Type config = readConfigA();
    config.COMP_TEMP_EN = enable;
    writeConfigA(config);
}

void LIS2MDL::rebootMemory()
{
    Config_A_Type config = readConfigA();
    config.REBOOT = 1;
    writeConfigA(config);
}

void LIS2MDL::softReset()
{
    Config_A_Type config = readConfigA();
    config.SOFT_RST = 1;
    writeConfigA(config);
}

void LIS2MDL::enableLowPower(bool enable)
{
    Config_A_Type config = readConfigA();
    config.REBOOT = enable;
    writeConfigA(config);
}

void LIS2MDL::setODR(ODR odr)
{
    Config_A_Type config = readConfigA();
    config.ODR = odr;
    writeConfigA(config);
}

ODR LIS2MDL::getODR()
{
    Config_A_Type config = readConfigA();
    return (ODR)config.ODR;
}

void LIS2MDL::setMode(ModeSetting mode)
{
    Config_A_Type config = readConfigA();
    config.MD = mode;
    writeConfigA(config);
}

ModeSetting LIS2MDL::getMode()
{
    Config_A_Type config = readConfigA();
    return (ModeSetting)config.MD;
}

// CFG_REG_B
Config_B_Type LIS2MDL::readConfigB()
{
    Config_B_Type config;
    uint8_t data = readByte(CFG_REG_B_ADDR);
    memcpy(&config, &data, sizeof(data));
    return config;
}

void LIS2MDL::setConfigB(Config_B_Type config)
{
    writeByte(CFG_REG_B_ADDR, (uint8_t *)&config);
}

void LIS2MDL::enable_Offs_Canc_One_Shot(bool enable)
{
    Config_B_Type config = readConfigB();
    config.OFF_CANC_ONE_SHOT = enable;
    setConfigB(config);
}
void LIS2MDL::setSetFreq(setFreq freq)
{
    Config_B_Type config = readConfigB();
    config.Set_FREQ = freq;
    setConfigB(config);
}
setFreq LIS2MDL::getSetFreq()
{
    Config_B_Type config = readConfigB();
    return (setFreq)config.Set_FREQ;
}

void LIS2MDL::enableOffsetCancelation(bool enable)
{
    Config_B_Type config = readConfigB();
    config.OFF_CANC = enable;
    setConfigB(config);
}

void LIS2MDL::enableLowPassFilter(bool enable)
{
    Config_B_Type config = readConfigB();
    config.LPF = enable;
    setConfigB(config);
}

// CFG_REG_C
Config_C_Type LIS2MDL::readConfigC()
{
    Config_C_Type config;
    uint8_t data = readByte(CFG_REG_C_ADDR);
    memcpy(&config, &data, sizeof(data));
    return config;
}

void LIS2MDL::setConfigC(Config_C_Type config)
{
    #ifndef ALLOW_DISABLE_I2C
    config.I2C_DIS = 0; // make sure I2C isnt disabled
    #endif
    writeByte(CFG_REG_C_ADDR, ((uint8_t *)&config));
}

void LIS2MDL::enableInturruptOnPin(bool enable)
{
    Config_C_Type config = readConfigC();
    config.INT_on_PIN = enable;
    setConfigC(config);
}
void LIS2MDL::disableI2CInterface(bool enable)
{
    // commented out just in case I try to call it
    // this would be big bad
    #ifndef ALLOW_DISABLE_I2C
    Config_C_Type config = readConfigC();
    config.I2C_DIS = enable;
    setConfigC(config);
    #endif
}

void LIS2MDL::enableAvoidReadError(bool enable)
{
    Config_C_Type config = readConfigC();
    config.BDU = enable;
    setConfigC(config);
}
void LIS2MDL::invertData(bool enable)
{
    Config_C_Type config = readConfigC();
    config.BLE = enable;
    setConfigC(config);
}
void LIS2MDL::enableSPI(bool enable)
{
    Config_C_Type config = readConfigC();
    config.WSPI = enable;
    setConfigC(config);
}
void LIS2MDL::enableSelfTest(bool enable)
{
    Config_C_Type config = readConfigC();
    config.Self_test = enable;
    setConfigC(config);
}
void LIS2MDL::enableDataReady(bool enable)
{
    Config_C_Type config = readConfigC();
    config.DRDY_on_PIN = enable;
    setConfigC(config);
}

// INT_CTRL_REG
Inturrupt_Control_Type LIS2MDL::readInturruptControl()
{
    Inturrupt_Control_Type control;
    uint8_t data = readByte(INT_CRTL_REG_ADDR);
    memcpy(&control, &data, sizeof(data));
    return control;
}

void LIS2MDL::setInturruptControl(Inturrupt_Control_Type control)
{
    writeByte(INT_CRTL_REG_ADDR, (uint8_t *)&control);
}

void LIS2MDL::enableXinturrupt(bool enable)
{
    Inturrupt_Control_Type control = readInturruptControl();
    control.XIEN = enable;
    setInturruptControl(control);
}

void LIS2MDL::enableYinturrupt(bool enable)
{
    Inturrupt_Control_Type control = readInturruptControl();
    control.YIEN = enable;
    setInturruptControl(control);
}

void LIS2MDL::enableZinturrupt(bool enable)
{
    Inturrupt_Control_Type control = readInturruptControl();
    control.ZIEN = enable;
    setInturruptControl(control);
}

void LIS2MDL::setIntPolarity(polarity Polarity)
{
    Inturrupt_Control_Type control = readInturruptControl();
    control.IEA = Polarity;
    setInturruptControl(control);
}

polarity LIS2MDL::getIntPolarity()
{
    Inturrupt_Control_Type control = readInturruptControl();
    return (polarity)control.IEA;
}

void LIS2MDL::setInturruptMode(intMode mode)
{
    Inturrupt_Control_Type control = readInturruptControl();
    control.IEL = mode;
    setInturruptControl(control);
}

intMode LIS2MDL::getInturruptMode()
{
    Inturrupt_Control_Type control = readInturruptControl();
    return (intMode)control.IEL;
}

void LIS2MDL::enableInturrupt(bool enable)
{
    Inturrupt_Control_Type control = readInturruptControl();
    control.IEN = enable;
    setInturruptControl(control);
}

// INT_SOURCE_REG
inturrupt_Source_Type LIS2MDL::readInturruptSource()
{
    inturrupt_Source_Type source;
    uint8_t Reg = readByte(INT_SOURCE_REG_ADDR);
    memcpy(&source, &Reg, sizeof(Reg));
    return (inturrupt_Source_Type)source;
}

void LIS2MDL::setInturruptSource(inturrupt_Source_Type config)
{
    writeByte(INT_CRTL_REG_ADDR, (uint8_t *)&config);
}

// INT_THS_L_REG, INT_THS_H_REG
void LIS2MDL::setInturruptThreshold(uint16_t value)
{
    writeByte(INT_THS_H_REG_ADDR, value >> 8);
    writeByte(INT_THS_L_REG_ADDR, value & 0xFFFF);
}

uint16_t LIS2MDL::getInturruptThreshold()
{
    uint8_t upper = readByte(INT_THS_H_REG_ADDR);
    uint8_t lower = readByte(INT_THS_L_REG_ADDR);
    return (upper << 8) | lower;
}

// STATUS_REG
Status_type LIS2MDL::readStatus()
{
    uint8_t raw = readByte(STATUS_REG_ADDR);
    Status_type status;
    memcpy(&status, &raw, sizeof(uint8_t));
    return status;
}

int16_t LIS2MDL::readXRaw()
{
    uint8_t upper = readByte(OUTX_H_REG_ADDR);
    uint8_t lower = readByte(OUTX_L_REG_ADDR);
    return (upper << 8) | lower;
}

int16_t LIS2MDL::readYRaw()
{
    uint8_t upper = readByte(OUTY_H_REG_ADDR);
    uint8_t lower = readByte(OUTY_L_REG_ADDR);
    return (upper << 8) | lower;
}
int16_t LIS2MDL::readZRaw()
{
    uint8_t upper = readByte(OUTZ_H_REG_ADDR);
    uint8_t lower = readByte(OUTZ_L_REG_ADDR);
    return (upper << 8) | lower;
}

int16_t LIS2MDL::readRawTemp()
{
    uint8_t upper = readByte(TEMP_OUT_H_REG_ADDR);
    uint8_t lower = readByte(TEMP_OUT_L_REG_ADDR);
    return (upper << 8) | lower;
}
float LIS2MDL::readTempC()
{
    return ((float)readRawTemp()) / 8;
}

float LIS2MDL::readTempF()
{
    return (readTempC() * 1.8) + 32;
}

void LIS2MDL::readBytes(uint8_t addr, uint8_t *data, uint8_t size)
{
    Wire.beginTransmission(COMPASS_DEVICEADDRESS);
    DEBUG_PRINTF("readI2C reg 0x%02x\n", addr)
    Wire.write(addr);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)COMPASS_DEVICEADDRESS, size);
    for (uint8_t i = 0; i < size; ++i)
    {
        data[i] = Wire.read();
        DEBUG_PRINTF(" <- data[%d]:0x%02x\n", i, data[i])
    }
}

uint8_t LIS2MDL::readByte(uint8_t addr)
{
    uint8_t data;
    readBytes(addr, &data, 1);
    DEBUG_PRINTF("read byte = %d\n", data)
    return data;
}

void LIS2MDL::writeBytes(uint8_t addr, uint8_t *data, uint8_t size)
{
    Wire.beginTransmission(COMPASS_DEVICEADDRESS);
    DEBUG_PRINTF("writeI2C reg 0x%02x\n", addr)
    Wire.write(addr);
    for (uint8_t i = 0; i < size; i++)
    {
        DEBUG_PRINTF(" -> data[%d]:0x%02x\n", i, data[i])
        Wire.write(data[i]);
    }
    Wire.endTransmission();
}

void LIS2MDL::writeByte(uint8_t addr, uint8_t data)
{
    DEBUG_PRINTF("write byte = %d\n", data)
    writeBytes(addr, &data, 1);
}

void LIS2MDL::writeByte(uint8_t addr, uint8_t *data)
{
    DEBUG_PRINTF("write byte = %d\n", *data)
    writeBytes(addr, data, 1);
}
