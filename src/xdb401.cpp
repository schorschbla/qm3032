#include <Wire.h>

#define XDB401_ADDRESS 0x6D
#define XDB401_PRESSURE_REG 0x06

// Wire.begin() must have begin called prior to this function

int ReadXdb401PressureValue(int *result)
{
    uint32_t sample = 0;
    
    Wire.beginTransmission(XDB401_ADDRESS);
    Wire.write(XDB401_PRESSURE_REG);
    Wire.endTransmission(false);
    Wire.requestFrom(XDB401_ADDRESS, 3);
    sample = (Wire.read() << 16) | (Wire.read() << 8) | Wire.read();
    Wire.endTransmission();

    *result = (sample & 0x800000) ? sample - 0x1000000 : sample;

    // TODO return proper error codes
    return 0;
}