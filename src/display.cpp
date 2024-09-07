#include "display.h"

Display::Display(uint32_t freqWrite, int16_t pinSclk, int16_t pinMosi, int16_t pinDc, int16_t pinCs, int16_t pinRst)
{
    lgfx::Bus_SPI::config_t busConfig = bus.config();
    busConfig.freq_write = freqWrite;
    busConfig.dma_channel = SPI_DMA_CH1;
    busConfig.pin_sclk = pinSclk;
    busConfig.pin_mosi = pinMosi;
    busConfig.pin_dc = pinDc;
    bus.config(busConfig);
    panel.setBus(&bus);

    lgfx::Panel_Device::config_t panelConfig = panel.config();
    panelConfig.pin_cs = pinCs;
    panelConfig.pin_rst = pinRst;
    panelConfig.invert = true;
    panelConfig.bus_shared = false;
    panel.config(panelConfig);

    setPanel(&panel); 

    lv_disp_draw_buf_init(&draw_buf, buf, NULL, 16384);
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = 240;
    disp_drv.ver_res = 240;
    disp_drv.flush_cb = flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.user_data = this;
}

lv_disp_drv_t &Display::lvglDriver() 
{
    return disp_drv;
}

void Display::flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    Display *display = static_cast<Display *>(disp->user_data);
    display->pushImageDMA(area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, (lgfx::rgb565_t *)&color_p->full);
    lv_disp_flush_ready(disp);
}