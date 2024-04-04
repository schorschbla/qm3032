#pragma once

#include <TFT_eSPI.h>

class Gauge 
{
public:
    Gauge(unsigned int x, unsigned int y, unsigned int radius, unsigned int thickness, unsigned int bgColor);

    void setValue(unsigned int start, unsigned int amount);
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

class GaugeScale
{
public:
    GaugeScale(unsigned int x, unsigned int y, unsigned int radius, unsigned int thickness, int start, unsigned int amount, unsigned int segments, unsigned int color, unsigned int bgColor);

    void draw(TFT_eSPI &tft, unsigned int space);
    void drawLabel(TFT_eSPI &tft, unsigned int pos, const char *label, int offsetX = 0, int offsetY = 0);

private:
    unsigned int x;
    unsigned int y;
    unsigned int radius;
    unsigned int thickness;
    unsigned int start;
    unsigned int amount;
    unsigned int segments;
    unsigned int color;
    unsigned int bgColor;
};