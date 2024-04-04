#include <algorithm> 
#include "gradient.h"

Gradient::Gradient(const unsigned int *rgb, const float *weights, unsigned char length) :
    rgb(rgb), weights(weights), length(length)
{
}

inline unsigned char mix(unsigned char left, unsigned char right, float ratio)
{
    return (unsigned char) std::min(255, (int)(left * ratio + right * (1.0 - ratio)));
}

inline unsigned int mixRgb(unsigned int left, unsigned int right, float ratio)
{
    return mix(left & 0xff, right & 0xff, ratio) |
        (mix((left >> 8) & 0xff, (right >> 8) & 0xff, ratio) << 8) |
        (mix((left >> 16) & 0xff, (right >> 16) & 0xff, ratio) << 16);
}

unsigned int Gradient::getRgb(float value) const
{
    unsigned int pos = 0;
    while (pos < length - 1 && value > weights[pos])
    {
        value -= weights[pos];
        pos++;
    }

    if (value > weights[pos])
    {
        return rgb[length - 1];
    }

    return mixRgb(rgb[pos + 1], rgb[pos], value / weights[pos]);
}