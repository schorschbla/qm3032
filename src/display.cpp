#include "display.h"

Display::Display(uint32_t freqWrite, int16_t pinSclk, int16_t pinMosi, int16_t pinDc, int16_t pinCs, int16_t pinRst) : excluded({0, 0, 0, 0})
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

void Display::flush(const lv_area_t *area, lv_color_t *color_p)
{
    lgfx::v1::pixelcopy_t pc = create_pc((lgfx::rgb565_t *)&color_p->full);
    pc.src_bitwidth = area->x2 - area->x1 + 1;

    if (area->y1 < 10)
    {
        Serial.printf("%x\n", lv_color_to32(color_p[25 * pc.src_bitwidth + pc.src_bitwidth / 2]));
    }

    startWrite();

    if ((excluded.x1 | excluded.x2 | excluded.y1 | excluded.y2) == 0)
    {
        panel.writeImage(area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, &pc, true);
    }
    else
    {
        if (area->y1 < excluded.y1)
        {
            panel.writeImage(area->x1, area->y1, area->x2 - area->x1 + 1, min(area->y2, excluded.y1) - area->y1 + 1, &pc, true);
        }

        if (area->y2 > excluded.y1 && area->y2 < excluded.y2)
        {
            int y = max(excluded.y1, area->y1);
            int h = min(area->y2, excluded.y2) - y + 1;

            if (area->x1 < excluded.x1)
            {
                pc.src_y = y - area->y1;
                pc.src_x = 0;
                panel.writeImage(area->x1, y, excluded.x1 - area->x1 + 1, h, &pc, true);
            }

            if (area->x2 > excluded.x2)
            {
                pc.src_y = y - area->y1;
                pc.src_x = excluded.x2 - area->x1;
                panel.writeImage(excluded.x2, y, area->x2 - excluded.x2 - 1, h, &pc, true);
            }
        }
    }

    endWrite();
    lv_disp_flush_ready(&disp_drv);
}

void Display::flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    static_cast<Display *>(disp->user_data)->flush(area, color_p);
}