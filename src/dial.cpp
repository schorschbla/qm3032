#include "dial.h"

Dial::Dial(unsigned int x, unsigned int y, unsigned int radius, unsigned int thickness, unsigned int bgColor) :
    x(x), y(y), 
    radius(radius), thickness(thickness), 
    color(0), start(__INT_MAX__), end(__INT_MAX__), 
    dirtyStart(__INT_MAX__), dirtyEnd(__INT_MAX__),
    bgColor(bgColor)
{   
}

inline unsigned int wrapAngle(int angle)
{
    return angle < 0 ? 360 + angle : angle;
}

void Dial::setValue(int start, int amount)
{
    if (amount == 0)
        amount = 1;

    if (amount < 0)
    {
        this->start = wrapAngle(start + amount);
        this->end = this->start - amount;
    }
    else
    {
        this->start = wrapAngle(start);
        this->end = this->start + amount;
    }
}

void Dial::setColor(unsigned int color)
{
    this->color = color;
}

void Dial::draw(TFT_eSPI &tft)
{
    if (dirtyStart != __INT_MAX__)
    {
        if (start > dirtyStart && start < dirtyEnd)
        {
          tft.drawArc(x, y, radius + thickness, radius, (dirtyStart - 1) % 360, (start + 1) % 360, bgColor, bgColor);
        }
        if (dirtyStart < end && dirtyEnd > end)
        {
          tft.drawArc(x, y, radius + thickness, radius, (end - 1) % 360, (dirtyEnd + 1) % 360, bgColor, bgColor);
        }
    }
     
    tft.drawSmoothArc(x, y, radius + thickness, radius, start % 360, end % 360, color, bgColor);

    dirtyStart = start;
    dirtyEnd = end;
}