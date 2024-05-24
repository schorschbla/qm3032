#include <Wire.h>
#include <Arduino.h>

#define XDB401_ADDRESS 0x6D
#define XDB401_PRESSURE_REG 0x06

#define NSA2860X_PCH_CONFIG1_REG    0xa4
#define NSA2860X_COMMAND_REG        0x30
#define NSA2860X_STATUS_REG         0x02
#define ODR_P_10HZ_50HZ_NOTCH       0x09

#define NSA2860X_EEPROM_LOCK_REG         0xd9

// Wire.begin() must have begin called prior to this function

uint8_t ReadReg(uint8_t reg)
{
    Wire.beginTransmission(XDB401_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    if (Wire.requestFrom(XDB401_ADDRESS, 1) != 1)
    {
         Wire.endTransmission();
         return 0;
    }
    uint8_t value = Wire.read();
    Wire.endTransmission();
    return value;
}


void WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t data[2];
    data[0] = reg;
    data[1] = value;
    Wire.beginTransmission(XDB401_ADDRESS);
    Wire.write(data, 2);
    Wire.endTransmission();
}


int ReadXdb401PressureValue(int *result)
{
    uint32_t sample = 0;
    
    Wire.beginTransmission(XDB401_ADDRESS);
    Wire.write(XDB401_PRESSURE_REG);
    Wire.endTransmission(false);
    if (Wire.requestFrom(XDB401_ADDRESS, 3) != 3)
    {
         Wire.endTransmission();
         return -1;
    }
    sample = (Wire.read() << 16) | (Wire.read() << 8) | Wire.read();
    Wire.endTransmission();

    if (sample == 0xfffff)
    {
        return -2;
    }

    *result = (sample & 0x800000) ? sample - 0x1000000 : sample;

    // TODO return proper error codes
    return 0;
}

void ReadRegs()
{
    Wire.beginTransmission(XDB401_ADDRESS);
    Wire.write(NSA2860X_COMMAND_REG);
    Wire.endTransmission(false);
    if (Wire.requestFrom(XDB401_ADDRESS, 1) != 1)
    {
         Wire.endTransmission();
         return;
    }
    uint8_t value = Wire.read();
    Wire.endTransmission();

    Serial.printf("Command: 0x%02x\n", ReadReg(NSA2860X_COMMAND_REG));
    Serial.printf("PCH Config1: 0x%02x\n", ReadReg(NSA2860X_PCH_CONFIG1_REG));
    Serial.printf("Status: 0x%02x\n", ReadReg(NSA2860X_STATUS_REG));
    delay(1000);
    Serial.printf("Command: 0x%02x\n", ReadReg(NSA2860X_COMMAND_REG));
    Serial.printf("PCH Config1: 0x%02x\n", ReadReg(NSA2860X_PCH_CONFIG1_REG));
    Serial.printf("Status: 0x%02x\n", ReadReg(NSA2860X_STATUS_REG));

int r;
    ReadXdb401PressureValue(&r);
    Serial-printf("value: %d\n", r);

    Serial.printf("Command: 0x%02x\n", ReadReg(NSA2860X_COMMAND_REG));
    Serial.printf("PCH Config1: 0x%02x\n", ReadReg(NSA2860X_PCH_CONFIG1_REG));
    Serial.printf("Status: 0x%02x\n", ReadReg(NSA2860X_STATUS_REG));
    Serial.printf("EEprom lock: 0x%02x\n", ReadReg(NSA2860X_EEPROM_LOCK_REG));

  //  WriteReg(NSA2860X_PCH_CONFIG1_REG, 0xa9);
  //  Serial.printf("PCH Config1: 0x%02x\n", ReadReg(NSA2860X_PCH_CONFIG1_REG));

    return;

    // START EEprom Programming
    Serial.printf("Set command 0\n");
   WriteReg(NSA2860X_COMMAND_REG, 0x00);
   delay(10);

    Serial.printf("Command: 0x%02x\n", ReadReg(NSA2860X_COMMAND_REG));
    Serial.printf("PCH Config1: 0x%02x\n", ReadReg(NSA2860X_PCH_CONFIG1_REG));
    Serial.printf("Status: 0x%02x\n", ReadReg(NSA2860X_STATUS_REG));

    Serial.printf("Wirte ODR Setting\n");
   WriteReg(NSA2860X_PCH_CONFIG1_REG, 0xa9);
   delay(10);


    Serial.printf("Command: 0x%02x\n", ReadReg(NSA2860X_COMMAND_REG));
    Serial.printf("PCH Config1: 0x%02x\n", ReadReg(NSA2860X_PCH_CONFIG1_REG));
    Serial.printf("Status: 0x%02x\n", ReadReg(NSA2860X_STATUS_REG));


    Serial.printf("Set eeprom programming mode\n");
   WriteReg(NSA2860X_COMMAND_REG, 0x33);
   delay(10);

    Serial.printf("Command: 0x%02x\n", ReadReg(NSA2860X_COMMAND_REG));
    Serial.printf("PCH Config1: 0x%02x\n", ReadReg(NSA2860X_PCH_CONFIG1_REG));
    Serial.printf("Status: 0x%02x\n", ReadReg(NSA2860X_STATUS_REG));


    Serial.printf("Write eeprom\n");

    // PROgram EEPROM
   WriteReg(0x6A, 0x1E);
      delay(100);




    Serial.printf("Command: 0x%02x\n", ReadReg(NSA2860X_COMMAND_REG));
    Serial.printf("PCH Config1: 0x%02x\n", ReadReg(NSA2860X_PCH_CONFIG1_REG));
    Serial.printf("Status: 0x%02x\n", ReadReg(NSA2860X_STATUS_REG));
    Serial.printf("EEprom lock: 0x%02x\n", ReadReg(NSA2860X_EEPROM_LOCK_REG));

}