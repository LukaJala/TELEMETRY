#pragma once

#include <stdint.h>

typedef struct {
    uint16_t reg;
    uint8_t  val;
} imx500_reg_t;

/*
 * IMX500 2028x1520 2x2 Binning Mode
 * Raspberry Pi AI Camera (Sony IMX500)
 * External clock: 24 MHz
 * MIPI: 2 lanes, RAW10, 888 Mbps/lane (444 MHz link frequency)
 *
 * Derived from the Raspberry Pi Linux kernel driver:
 *   drivers/media/i2c/imx500.c (rpi-6.6.y branch)
 *
 * Sequence: standby → common clock/lane config → mode registers.
 * Stream enable/disable is handled separately via register 0x0100.
 */
static const imx500_reg_t imx500_2028x1520_30fps[] = {

    /* ------------------------------------------------------------------ */
    /* Enter standby before programming                                     */
    /* ------------------------------------------------------------------ */
    {0x0100, 0x00},

    /* ------------------------------------------------------------------ */
    /* Common: PLL pre-dividers                                             */
    /* ------------------------------------------------------------------ */
    {0x0305, 0x02},
    {0x0306, 0x00},
    {0x030d, 0x02},
    {0x030e, 0x00},

    /* Fast standby control */
    {0x0106, 0x01},

    /* External clock frequency: 24 MHz (integer part in Sony fixed-point) */
    {0x0136, 0x18},
    {0x0137, 0x00},

    /* ------------------------------------------------------------------ */
    /* Common: CSI output format and lane configuration                     */
    /* ------------------------------------------------------------------ */
    {0x0112, 0x0a},   /* CSI_DT_FMT_H: RAW10 */
    {0x0113, 0x0a},   /* CSI_DT_FMT_L */
    {0x0114, 0x01},   /* CSI_LANE_MODE: 2 data lanes */

    /* ------------------------------------------------------------------ */
    /* Mode: line and frame timing                                          */
    /* ------------------------------------------------------------------ */
    {0x0342, 0x24},   /* LINE_LENGTH_PCK MSB → 0x24b6 = 9398 */
    {0x0343, 0xb6},   /* LINE_LENGTH_PCK LSB */
    {0x0340, 0x0b},   /* FRM_LENGTH_LINES MSB → 0x0b9c = 2972 */
    {0x0341, 0x9c},   /* FRM_LENGTH_LINES LSB */
    {0x3210, 0x00},   /* LONG_EXP_SHIFT */

    /* ------------------------------------------------------------------ */
    /* Mode: crop window (full 4056×3040 array)                            */
    /* ------------------------------------------------------------------ */
    {0x0344, 0x00},   /* X_ADD_STA_A MSB → left = 0 */
    {0x0345, 0x00},   /* X_ADD_STA_A LSB */
    {0x0346, 0x00},   /* Y_ADD_STA_A MSB → top = 0 */
    {0x0347, 0x00},   /* Y_ADD_STA_A LSB */
    {0x0348, 0x0f},   /* X_ADD_END_A MSB → 0x0fd7 = 4055 */
    {0x0349, 0xd7},   /* X_ADD_END_A LSB */
    {0x0350, 0x00},   /* digital crop enable */
    {0x034a, 0x0b},   /* Y_ADD_END_A MSB → 0x0bdf = 3039 */
    {0x034b, 0xdf},   /* Y_ADD_END_A LSB */

    /* ------------------------------------------------------------------ */
    /* Mode: 2×2 binning                                                   */
    /* ------------------------------------------------------------------ */
    {0x3f58, 0x01},
    {0x0381, 0x01},   /* X_EVN_INC */
    {0x0383, 0x01},   /* X_ODD_INC */
    {0x0385, 0x01},   /* Y_EVN_INC */
    {0x0387, 0x01},   /* Y_ODD_INC */
    {0x0900, 0x01},   /* BINNING_MODE: enabled */
    {0x0901, 0x22},   /* BINNING_TYPE: 2×2 */
    {0x0902, 0x02},   /* BINNING_WEIGHTING */

    /* ------------------------------------------------------------------ */
    /* Mode: internal sensor tuning                                         */
    /* ------------------------------------------------------------------ */
    {0x3241, 0x11},
    {0x3242, 0x01},
    {0x3250, 0x03},
    {0x3f0f, 0x00},
    {0x3f40, 0x00},
    {0x3f41, 0x00},
    {0x3f42, 0x00},
    {0x3f43, 0x00},
    {0xb34e, 0x00},
    {0xb351, 0x20},
    {0xb35c, 0x00},
    {0xb35e, 0x08},

    /* ------------------------------------------------------------------ */
    /* Mode: digital crop / scaler (no scaling)                            */
    /* ------------------------------------------------------------------ */
    {0x0401, 0x00},   /* SCALE_MODE: disabled */
    {0x0404, 0x00},   /* SCALE_M MSB */
    {0x0405, 0x10},   /* SCALE_M LSB (1.0×) */
    {0x0408, 0x00},   /* DIG_CROP_X_OFFSET MSB */
    {0x0409, 0x00},   /* DIG_CROP_X_OFFSET LSB */
    {0x040a, 0x00},   /* DIG_CROP_Y_OFFSET MSB */
    {0x040b, 0x00},   /* DIG_CROP_Y_OFFSET LSB */
    {0x040c, 0x07},   /* DIG_CROP_IMAGE_WIDTH MSB → 2028 */
    {0x040d, 0xec},   /* DIG_CROP_IMAGE_WIDTH LSB */
    {0x040e, 0x05},   /* DIG_CROP_IMAGE_HEIGHT MSB → 1520 */
    {0x040f, 0xf0},   /* DIG_CROP_IMAGE_HEIGHT LSB */

    /* ------------------------------------------------------------------ */
    /* Mode: output size                                                    */
    /* ------------------------------------------------------------------ */
    {0x034c, 0x07},   /* X_OUTPUT_SIZE MSB → 2028 */
    {0x034d, 0xec},   /* X_OUTPUT_SIZE LSB */
    {0x034e, 0x05},   /* Y_OUTPUT_SIZE MSB → 1520 */
    {0x034f, 0xf0},   /* Y_OUTPUT_SIZE LSB */

    /* ------------------------------------------------------------------ */
    /* Mode: PLL / pixel clock                                             */
    /* ------------------------------------------------------------------ */
    {0x0301, 0x05},   /* VTPXCK_DIV */
    {0x0303, 0x02},   /* VTSYCK_DIV */
    {0x0307, 0x9b},   /* PLL_MPY = 155 */
    {0x0309, 0x0a},   /* OPPXCK_DIV */
    {0x030b, 0x01},   /* OPSYCK_DIV */
    {0x030f, 0x4a},   /* PLL_MPY2 */
    {0x0310, 0x01},   /* PLL_MODE */

    /* ------------------------------------------------------------------ */
    /* Mode: MIPI output bit-rate request                                  */
    /* 0x07ce = 1998 → ~999 Mbps/lane (matches 444 MHz link freq × 2 DDR) */
    /* ------------------------------------------------------------------ */
    {0x0820, 0x07},   /* REQ_LINK_BIT_RATE_MBPS [31:24] */
    {0x0821, 0xce},   /* REQ_LINK_BIT_RATE_MBPS [23:16] */
    {0x0822, 0x00},   /* REQ_LINK_BIT_RATE_MBPS [15:8] */
    {0x0823, 0x00},   /* REQ_LINK_BIT_RATE_MBPS [7:0] */

    /* ------------------------------------------------------------------ */
    /* Mode: internal tuning                                                */
    /* ------------------------------------------------------------------ */
    {0x3e20, 0x01},
    {0x3e35, 0x01},
    {0x3e36, 0x01},
    {0x3e37, 0x00},
    {0x3e3a, 0x01},
    {0x3e3b, 0x00},

    /* ------------------------------------------------------------------ */
    /* Mode: HDR / exposure control (single exposure mode)                 */
    /* ------------------------------------------------------------------ */
    {0x00e3, 0x00},
    {0x00e4, 0x00},
    {0x00e6, 0x00},
    {0x00e7, 0x00},
    {0x00e8, 0x00},
    {0x00e9, 0x00},

    /* ------------------------------------------------------------------ */
    /* Mode: additional internal tuning                                     */
    /* ------------------------------------------------------------------ */
    {0x3f50, 0x00},
    {0x3f56, 0x01},
    {0x3f57, 0x30},
    {0x3606, 0x01},
    {0x3607, 0x01},
    {0x3f26, 0x00},
    {0x3f4a, 0x00},
    {0x3f4b, 0x00},
    {0x4bc0, 0x16},
    {0x7ba8, 0x00},
    {0x7ba9, 0x00},
    {0x886b, 0x00},
    {0x579a, 0x00},
    {0x579b, 0x0a},
    {0x579c, 0x01},
    {0x579d, 0x2a},
    {0x57ac, 0x00},
    {0x57ad, 0x00},
    {0x57ae, 0x00},
    {0x57af, 0x81},
    {0x57be, 0x00},
    {0x57bf, 0x00},
    {0x57c0, 0x00},
    {0x57c1, 0x81},
    {0x57d0, 0x00},
    {0x57d1, 0x00},
    {0x57d2, 0x00},
    {0x57d3, 0x81},
    {0x5324, 0x00},
    {0x5325, 0x31},
    {0x5326, 0x00},
    {0x5327, 0x60},
    {0xbca7, 0x08},
    {0x5fcc, 0x1e},
    {0x5fd7, 0x1e},
    {0x5fe2, 0x1e},
    {0x5fed, 0x1e},
    {0x5ff8, 0x1e},
    {0x6003, 0x1e},
    {0x5d0b, 0x02},
    {0x6f6d, 0x01},
    {0x61c9, 0x68},
    {0x5352, 0x00},
    {0x5353, 0x3f},
    {0x5356, 0x00},
    {0x5357, 0x1c},
    {0x5358, 0x00},
    {0x5359, 0x3d},
    {0x535c, 0x00},
    {0x535d, 0xa6},
    {0x6187, 0x1d},
    {0x6189, 0x1d},
    {0x618b, 0x1d},
    {0x618d, 0x23},
    {0x618f, 0x23},
    {0x5414, 0x01},
    {0x5415, 0x12},
    {0xbca8, 0x00},
    {0x5fcf, 0x28},
    {0x5fda, 0x2d},
    {0x5fe5, 0x2d},
    {0x5ff0, 0x2d},
    {0x5ffb, 0x2d},
    {0x6006, 0x2d},
    {0x616e, 0x04},
    {0x616f, 0x04},
    {0x6170, 0x04},
    {0x6171, 0x06},
    {0x6172, 0x06},
    {0x6173, 0x0c},
    {0x6174, 0x0c},
    {0x6175, 0x0c},
    {0x6176, 0x00},
    {0x6177, 0x10},
    {0x6178, 0x00},
    {0x6179, 0x1a},
    {0x617a, 0x00},
    {0x617b, 0x1a},
    {0x617c, 0x00},
    {0x617d, 0x27},
    {0x617e, 0x00},
    {0x617f, 0x27},
    {0x6180, 0x00},
    {0x6181, 0x44},
    {0x6182, 0x00},
    {0x6183, 0x44},
    {0x6184, 0x00},
    {0x6185, 0x44},
    {0x5dfc, 0x0a},
    {0x5e00, 0x0a},
    {0x5e04, 0x0a},
    {0x5e08, 0x0a},
    {0x5dfd, 0x0a},
    {0x5e01, 0x0a},
    {0x5e05, 0x0a},
    {0x5e09, 0x0a},
    {0x5dfe, 0x0a},
    {0x5e02, 0x0a},
    {0x5e06, 0x0a},
    {0x5e0a, 0x0a},
    {0x5dff, 0x0a},
    {0x5e03, 0x0a},
    {0x5e07, 0x0a},
    {0x5e0b, 0x0a},
    {0x5dec, 0x12},
    {0x5df0, 0x12},
    {0x5df4, 0x21},
    {0x5df8, 0x31},
    {0x5ded, 0x12},
    {0x5df1, 0x12},
    {0x5df5, 0x21},
    {0x5df9, 0x31},
    {0x5dee, 0x12},
    {0x5df2, 0x12},
    {0x5df6, 0x21},
    {0x5dfa, 0x31},
    {0x5def, 0x12},
    {0x5df3, 0x12},
    {0x5df7, 0x21},
    {0x5dfb, 0x31},
    {0x5ddc, 0x0d},
    {0x5de0, 0x0d},
    {0x5de4, 0x0d},
    {0x5de8, 0x0d},
    {0x5ddd, 0x0d},
    {0x5de1, 0x0d},
    {0x5de5, 0x0d},
    {0x5de9, 0x0d},
    {0x5dde, 0x0d},
    {0x5de2, 0x0d},
    {0x5de6, 0x0d},
    {0x5dea, 0x0d},
    {0x5ddf, 0x0d},
    {0x5de3, 0x0d},
    {0x5de7, 0x0d},
    {0x5deb, 0x0d},
    {0x5dcc, 0x55},
    {0x5dd0, 0x50},
    {0x5dd4, 0x4b},
    {0x5dd8, 0x4b},
    {0x5dcd, 0x55},
    {0x5dd1, 0x50},
    {0x5dd5, 0x4b},
    {0x5dd9, 0x4b},
    {0x5dce, 0x55},
    {0x5dd2, 0x50},
    {0x5dd6, 0x4b},
    {0x5dda, 0x4b},
    {0x5dcf, 0x55},
    {0x5dd3, 0x50},
    {0x5dd7, 0x4b},
    {0x5ddb, 0x4b},

    /* ------------------------------------------------------------------ */
    /* Mode: default exposure and gain                                      */
    /* ------------------------------------------------------------------ */
    {0x0202, 0x0b},   /* COARSE_INTEG_TIME MSB → 0x0b86 = 2950 lines */
    {0x0203, 0x86},   /* COARSE_INTEG_TIME LSB */
    {0x0204, 0x00},   /* ANA_GAIN_GLOBAL MSB */
    {0x0205, 0x00},   /* ANA_GAIN_GLOBAL LSB */
    {0x020e, 0x01},   /* DIG_GAIN_GR MSB */
    {0x020f, 0x00},   /* DIG_GAIN_GR LSB */
    {0x0210, 0x01},   /* DIG_GAIN_R MSB */
    {0x0211, 0x00},   /* DIG_GAIN_R LSB */
    {0x0212, 0x01},   /* DIG_GAIN_B MSB */
    {0x0213, 0x00},   /* DIG_GAIN_B LSB */
    {0x0214, 0x01},   /* DIG_GAIN_GB MSB */
    {0x0215, 0x00},   /* DIG_GAIN_GB LSB */
};
