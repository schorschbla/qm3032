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

lv_area_t excluded = {
    56,
    112,
    184,
    240
};

void Display::flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    Display *display = static_cast<Display *>(disp->user_data);
    lgfx::v1::pixelcopy_t pc = display->create_pc((lgfx::rgb565_t *)&color_p->full);
    pc.src_bitwidth = area->x2 - area->x1 + 1;

    display->startWrite();

    if (area->y1 < excluded.y1)
    {
        display->panel.writeImage(area->x1, area->y1, area->x2 - area->x1 + 1, min(area->y2, excluded.y1) - area->y1 + 1, &pc, true);
    }

    if (area->y2 > excluded.y1 && area->y2 < excluded.y2)
    {
        int y = max(excluded.y1, area->y1);
        int h = min(area->y2, excluded.y2) - y + 1;

        if (area->x1 < excluded.x1)
        {
            pc.src_y = y - area->y1;
            pc.src_x = 0;
            display->panel.writeImage(area->x1, y, excluded.x1 - area->x1 + 1, h, &pc, true);
        }

        if (area->x2 > excluded.x2)
        {
            pc.src_y = y - area->y1;
            pc.src_x = excluded.x2 - area->x1;
            display->panel.writeImage(excluded.x2, y, area->x2 - excluded.x2 - 1, h, &pc, true);
        }
    }

    display->endWrite();
    lv_disp_flush_ready(disp);
}