#pragma once

#define LCD_H_RES 800
#define LCD_V_RES 1280
/* RGB565 (16bpp): halves framebuffer PSRAM bandwidth vs RGB888. The DPI scan-out
 * is a constant ~160MB/s drain at 24bpp; 16bpp cuts it to ~107MB/s, freeing the
 * memory bus for the camera capture/PPA path (smoother video). LVGL is already
 * built at color depth 16, so this also drops a per-flush RGB565->RGB888 convert. */
#define LCD_BIT_PER_PIXEL 16
#define MIPI_DSI_LANE_NUM 2

#define PIN_NUM_LCD_RST 27
#define PIN_NUM_BK_LIGHT 26
#define LCD_BK_LIGHT_ON_LEVEL 1

#define MIPI_DSI_PHY_PWR_LDO_CHAN 3
#define MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500
