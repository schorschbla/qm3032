#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lvgl.h>

class Display : public lgfx::LGFX_Device
{
public:
  Display(uint32_t freqWrite = 30000000, int16_t pinSclk = 26, int16_t pinMosi = 14, int16_t pinDc = 27, int16_t pinCs = 12, int16_t pinRst = 13);

  lv_disp_drv_t &lvglDriver();

  inline void setLvglExlucdedArea(const lv_area_t &excluded)
  {
    this->excluded = excluded;
  }

  inline void clearLvglExcludedArea()
  {
    excluded = {0, 0, 0, 0};
  }

private:
  lgfx::Bus_SPI bus;
  lgfx::Panel_GC9A01 panel;
  lv_color_t buf[16384];
  lv_disp_draw_buf_t draw_buf;
  lv_disp_drv_t disp_drv;
  lv_area_t excluded;

  void flush(const lv_area_t *area, lv_color_t *color_p);
  static void flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
};