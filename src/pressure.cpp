#include <Wire.h>
#include <math.h>

#define SENSOR_ADDRESS 0x6D
#define SENSOR_REG 0x06

int compare(const void *cmp1, const void *cmp2)
{
    uint32_t a = *((uint32_t *)cmp1);
    uint32_t b = *((uint32_t *)cmp2);
    return a > b ? -1 : (a < b ? 1 : 0);
}

float getPressureValue()
{
    uint32_t samples[16];
 
    int c = 0;
    for (int i = 0; c < 16; ++i)
    {
        if (i > 255)
            return NAN;

        uint32_t sample = 0;
        Wire.beginTransmission(SENSOR_ADDRESS);
        Wire.write(SENSOR_REG);
        Wire.endTransmission(false);
        Wire.requestFrom(SENSOR_ADDRESS, 3);
        if (Wire.available() == 3)
        {
            sample = (Wire.read() << 16) | (Wire.read() << 8) | Wire.read();
        }
        Wire.endTransmission();

        if (sample != 0 && (sample & 0x800000) == 0)
        {
            samples[c++] = sample;
            delayMicroseconds(100);
        }
    }

    qsort(samples, 8, sizeof(uint32_t), compare);

    float value = 0;
    for (int i = 4; i < 12; ++i)
    {
        value += samples[i];
    }

    return value / 3355443.2;
}