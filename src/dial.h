#pragma once

#include <TFT_eSPI.h>

class Dial 
{
public:
    Dial(unsigned int x, unsigned int y, unsigned int radius, unsigned int thickness, unsigned int bgColor);

    void setValue(int start, int amount);
    void setColor(unsigned int color);
    void draw(TFT_eSPI &tft);
    bool isDirty() const;

private:
    unsigned int x;
    unsigned int y;
    unsigned int radius;
    unsigned int thickness;
    unsigned int bgColor;
    unsigned int color;

    unsigned int start;
    unsigned int end;

    unsigned int dirtyStart;
    unsigned int dirtyEnd;
    unsigned int dirtyColor;
};