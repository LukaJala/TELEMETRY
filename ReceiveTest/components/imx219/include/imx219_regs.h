#pragma once

#include <stdint.h>

typedef struct {
    uint16_t reg;
    uint8_t val;
} imx219_reg_t;

/*
 * IMX219 1536x1232 2x2 Binning (Full FOV)
 * 24 MHz external clock, 2 MIPI lanes, RAW10
 * MIPI clock: 456 MHz (912 Mbps/lane)
 */
static const imx219_reg_t imx219_1536x1232_30fps[] = {
    {0x0100, 0x00}, /* standby */

    /* Special access sequence */
    {0x30EB, 0x05}, {0x30EB, 0x0C}, {0x300A, 0xFF}, {0x300B, 0xFF},
    {0x30EB, 0x05}, {0x30EB, 0x09},

    /* PLL — 912 Mbps/lane, 182.4 MHz pixel clock */
    {0x0301, 0x05}, /* VTPXCK_DIV  */
    {0x0303, 0x01}, /* VTSYCK_DIV  */
    {0x0304, 0x03}, /* PREPLLCK_VT_DIV */
    {0x0305, 0x03}, /* PREPLLCK_OP_DIV */
    {0x0306, 0x00}, /* PLL_VT_MPY hi */
    {0x0307, 0x39}, /* PLL_VT_MPY lo = 57 */
    {0x0309, 0x0A}, /* OPPXCK_DIV  */
    {0x030B, 0x01}, /* OPSYCK_DIV  */
    {0x030C, 0x00}, /* PLL_OP_MPY hi */
    {0x030D, 0x72}, /* PLL_OP_MPY lo = 114 */

    /* CSI-2 */
    {0x0114, 0x01}, /* 2 lanes */
    {0x0128, 0x01}, /* continuous clock */
    {0x012A, 0x18}, /* EXCK_FREQ = 24 MHz */
    {0x012B, 0x00},

    /* Frame timing */
    {0x0160, 0x06}, {0x0161, 0xE3}, /* frame length = 1763 */
    {0x0162, 0x0D}, {0x0163, 0x78}, /* line length  = 3448 */

    /* Full sensor window 3280x2464 */
    {0x0164, 0x00}, {0x0165, 0x00}, /* X start = 0    */
    {0x0166, 0x0C}, {0x0167, 0xCF}, /* X end   = 3279 */
    {0x0168, 0x00}, {0x0169, 0x00}, /* Y start = 0    */
    {0x016A, 0x09}, {0x016B, 0x9F}, /* Y end   = 2463 */

    /* Output size 1536x1232 (2x2 binning, 64-byte aligned) */
    {0x016C, 0x06}, {0x016D, 0x00}, /* width  = 1536 */
    {0x016E, 0x04}, {0x016F, 0xD0}, /* height = 1232 */

    /* 2x2 analog binning */
    {0x0174, 0x03}, {0x0175, 0x03},

    /* RAW10 */
    {0x018C, 0x0A}, {0x018D, 0x0A},

    /* Default exposure & gain */
    {0x0157, 0xE8},             /* analogue gain (max x10.5) */
    {0x015A, 0x06}, {0x015B, 0xD6}, /* coarse integration = 1750 lines */
};
