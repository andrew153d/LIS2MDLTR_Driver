#ifndef LIS2MDL_H
#define LIS2MDL_H

#include <Arduino.h>
#include <Wire.h>



#define OFFSET_X_REG_L_ADDR 0x45
#define OFFSET_X_REG_H_ADDR 0x46
#define OFFSET_Y_REG_L_ADDR 0x47
#define OFFSET_Y_REG_H_ADDR 0x48
#define OFFSET_Z_REG_L_ADDR 0x49
#define OFFSET_Z_REG_H_ADDR 0x4A
#define RESERVED_4B_ADDR 0x4B
#define RESERVED_4C_ADDR 0x4C
#define WHO_AM_I_ADDR 0x4F
#define RESERVED_50_ADDR 0x50
#define RESERVED_5F_ADDR 0x5F
#define CFG_REG_A_ADDR 0x60
#define CFG_REG_B_ADDR 0x61
#define CFG_REG_C_ADDR 0x62
#define INT_CRTL_REG_ADDR 0x63
#define INT_SOURCE_REG_ADDR 0x64
#define INT_THS_L_REG_ADDR 0x65
#define INT_THS_H_REG_ADDR 0x66
#define STATUS_REG_ADDR 0x67
#define OUTX_L_REG_ADDR 0x68
#define OUTX_H_REG_ADDR 0x69
#define OUTY_L_REG_ADDR 0x6A
#define OUTY_H_REG_ADDR 0x6B
#define OUTZ_L_REG_ADDR 0x6C
#define OUTZ_H_REG_ADDR 0x6D
#define TEMP_OUT_L_REG_ADDR 0x6E
#define TEMP_OUT_H_REG_ADDR 0x6F

#define WHOAMI_VAL 0b01000000
#define SENSITIVITY_MGAUSS_PER_LSB 1.5
#define COMPASS_DEVICEADDRESS 0x1E

#define DEBUG

//COMP_TEMP_EN | REBOOT | SOFT_RST | LP | ODR[1:0] | MD[1:0]
#define DEFAULT_CONFIG_A 0b10001100

// 0 | 0 | 0 | OFF_CANC_ONE_SHOT | INT_on_DATAOFF | Set_Freq | OFF_CANC | LPF
#define DEFAULT_CONFIG_B 0b00000001

// 0 | INT_ON_PIN | I2C_DIS | BDU | BLE | 4WSPI | Self_Test | DRDY_on_PIN
#define DEFAULT_CONFIG_C 0b00000001

#define LSB_TO_MSG 1.5
//#define LIS2MDL_DEBUG

#define DEBUG_PRINTER Serial

#ifdef LIS2MDL_DEBUG
#define DEBUG_PRINT(...)                                            \
    {                                                               \
        DEBUG_PRINTER.printf("[LIS2MDL(0x%02x)]: ", COMPASS_DEVICEADDRESS); \
        DEBUG_PRINTER.print(__VA_ARGS__);                           \
    }
#define DEBUG_PRINTLN(...)                                          \
    {                                                               \
        DEBUG_PRINTER.printf("[LIS2MDL(0x%02x)]: ", COMPASS_DEVICEADDRESS); \
        DEBUG_PRINTER.println(__VA_ARGS__);                         \
    }
#define DEBUG_PRINTF(...)                                           \
    {                                                               \
        DEBUG_PRINTER.printf("[LIS2MDL(0x%02x)]: ", COMPASS_DEVICEADDRESS); \
        DEBUG_PRINTER.printf(__VA_ARGS__);                          \
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
#endif

// CFG_REG_A
#pragma pack(push, 1)
struct Config_A_Type
{
    unsigned int COMP_TEMP_EN : 1;
    unsigned int REBOOT : 1;
    unsigned int SOFT_RST : 1;
    unsigned int LP : 1;
    unsigned int ODR : 1;
    unsigned int MD : 2;
};
#pragma pack(pop)

enum ODR : uint8_t
{
    ODR_10_Hz = 0,
    ODR_20_Hz = 1,
    ODR_50_Hz = 2,
    ODR_100_Hz = 3
};

enum ModeSetting : uint8_t
{
    ContinuousMode = 0,
    SingleMode = 1,
    IdleMode1 = 2,
    IdleMode2 = 3
};

// CFG_REG_B
#pragma pack(push, 1)
struct Config_B_Type
{
    unsigned int empty : 3;
    unsigned int OFF_CANC_ONE_SHOT : 1;
    unsigned int INT_on_DataOFF : 1;
    unsigned int Set_FREQ : 1;
    unsigned int OFF_CANC : 1;
    unsigned int LPF : 1;
};
#pragma pack(pop)

enum setFreq : bool
{
    EVERY_63_ODR = 0,
    AT_POWER_ON = 1
};

// CFG_REG_C
#pragma pack(push, 1) // Push current alignment to stack, set packing to 1 byte

struct Config_C_Type
{
    unsigned int empty : 1;
    unsigned int INT_on_PIN : 1;
    unsigned int I2C_DIS : 1;
    unsigned int BDU : 1;
    unsigned int BLE : 1;
    unsigned int WSPI : 1;
    unsigned int Self_test : 1;
    unsigned int DRDY_on_PIN : 1;
};

// INT_CTR_REG
struct Inturrupt_Control_Type
{
    unsigned int XIEN : 1;
    unsigned int YIEN : 1;
    unsigned int ZIEN : 1;
    unsigned int Reserved1 : 1;
    unsigned int Reserved2 : 1;
    unsigned int IEA : 1;
    unsigned int IEL : 1;
    unsigned int IEN : 1;
};

#pragma pack(pop) // Restore original alignment from stack

enum polarity
{
    INT_HIGH = 1,
    INT_LOW = 0
};

enum intMode
{
    PULSE = 0,
    LATCHED = 1
};

// INT_SOURCE_REG
struct inturrupt_Source_Type
{
    unsigned int P_TH_S_X : 1;
    unsigned int P_TH_S_Y : 1;
    unsigned int P_TH_S_Z : 1;
    unsigned int N_TH_S_X : 1;
    unsigned int N_TH_S_Y : 1;
    unsigned int N_TH_S_Z : 1;
    unsigned int MROI : 1;
    unsigned int INT : 1;
};

// STATUS_REG
struct Status_type
{
    unsigned int XYZ_Overrun : 1;
    unsigned int X_Overrun : 1;
    unsigned int Y_Overrun : 1;
    unsigned int Z_Overrun : 1;
    unsigned int XYZ_available : 1;
    unsigned int X_available : 1;
    unsigned int Y_available : 1;
    unsigned int Z_available : 1;
};

class LIS2MDL
{
private:
    float X, Y, Z;
public:
    LIS2MDL();

    bool begin();

    void readMag(float& X, float&Y, float&Z);
    void readRaw(int16_t& X, int16_t&Y, int16_t&Z);

    void calibrate();

    float getHeading();

    void writeXOffset(int16_t setVal);
    int16_t readXOffset();
    void writeYOffset(int16_t setVal);
    int16_t readYOffset();
    void writeZOffset(int16_t setVal);
    int16_t readZOffset();

    // CFG_REG_A
    Config_A_Type readConfigA();
    void writeConfigA(Config_A_Type config);
    void enableTempComp(bool enable = true);
    void rebootMemory();
    void softReset();
    void enableLowPower(bool enable = true);
    void setODR(ODR odr);
    ODR getODR();
    void setMode(ModeSetting mode);
    ModeSetting getMode();

    // CFG_REG_B
    Config_B_Type readConfigB();
    void setConfigB(Config_B_Type);
    void enable_Offs_Canc_One_Shot(bool enable = true);
    void setSetFreq(setFreq);
    setFreq getSetFreq();
    void enableOffsetCancelation(bool enable = true);
    void enableLowPassFilter(bool enable = true);

    // CFG_REG_C
    Config_C_Type readConfigC();
    void setConfigC(Config_C_Type);
    void enableInturruptOnPin(bool enable = true);
    void disableI2CInterface(bool enable = true);
    void enableAvoidReadError(bool enable = true);
    void invertData(bool enable = true);
    void enableSPI(bool enable = true);
    void enableSelfTest(bool enable = true);
    void enableDataReady(bool enable = true);

    // INT_CTRL_REG
    Inturrupt_Control_Type readInturruptControl();
    void setInturruptControl(Inturrupt_Control_Type control);
    void enableXinturrupt(bool enable = true);
    void enableYinturrupt(bool enable = true);
    void enableZinturrupt(bool enable = true);
    void setIntPolarity(polarity Polarity);
    polarity getIntPolarity();
    void setInturruptMode(intMode);
    intMode getInturruptMode();
    void enableInturrupt(bool enable);
    void enableINTBit(bool enable = true);

    // INT_SOURCE_REG
    inturrupt_Source_Type readInturruptSource();
    void setInturruptSource(inturrupt_Source_Type config);

    // INT_THS_L_REG, INT_THS_H_REG
    void setInturruptThreshold(uint16_t value);
    uint16_t getInturruptThreshold();

    // STATUS_REG
    Status_type readStatus();

    int16_t readXRaw();
    int16_t readYRaw();
    int16_t readZRaw();

    int16_t readRawTemp();
    float readTempC();
    float readTempF();

    void readBytes(uint8_t addr, uint8_t *data, uint8_t size);
    uint16_t readWord(uint8_t addr);
    uint8_t readByte(uint8_t addr);
    void writeBytes(uint8_t addr, uint8_t *data, uint8_t size);
    void writeWord(uint8_t addr, uint16_t data);
    void writeByte(uint8_t addr, uint8_t data);
    void writeByte(uint8_t addr, uint8_t *data);
};

#endif