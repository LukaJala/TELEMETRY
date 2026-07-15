/*
 * camera_uvc.c
 * USB UVC webcam (e.g. Logitech C920) backend — always-on backup camera.
 *
 * Selected by CONFIG_CAMERA_BACKEND_UVC. Implements the exact same camera.h
 * API as camera_csi.c, so main.c is identical either way. Everything downstream
 * of "here is an RGB565 frame" — the PPA scale/rotate, the PiP crop, the
 * fullscreen zero-copy into the DPI frame buffer, and the LVGL-lock freeze — is
 * a deliberate mirror of camera_csi.c so the two backends behave identically on
 * screen. Only the frame *source* differs.
 *
 * Pipeline: C920 (MJPEG over UVC, High-Speed USB OTG)
 *             -> usb_host_uvc frame callback (driver task)
 *             -> hardware JPEG decode (MJPEG -> RGB565, full frame in PSRAM)
 *             -> PPA scale (+rotate) -> one of two output targets:
 *
 *   PiP (default):  centered vertical crop scaled into the private LVGL-canvas
 *                   buffer, unrotated (LVGL's sw-rotate orients it at flush).
 *   Fullscreen:     full decoded frame rotated 90deg directly into the DPI
 *                   frame buffer while the processing task holds the LVGL lock,
 *                   freezing the dashboard; on exit it invalidates the screen.
 *
 * Fullscreen is requested via camera_set_reverse() (gear = REV) or
 * camera_set_fullscreen_manual() (__CAM_START__/__CAM_STOP__); effective mode =
 * manual OR reverse, applied at frame boundaries. Same contract as the CSI path.
 *
 * =====================================================================
 * BENCH BRING-UP — verify these before trusting the image (see CLAUDE.md):
 *   1. VBUS: the P4 host port must supply ~500 mA @ 5V for the C920. Use a
 *      powered hub / external 5V on the bench if the camera browns out.
 *   2. VID/PID: leave CONFIG_CAMERA_UVC_VID/PID at 0 first, read the real
 *      IDs from the "UVC device ..." enumeration log, then pin them.
 *   3. This file is written against the espressif/usb_host_uvc v2.x API and
 *      driver/jpeg_decode.h (IDF 5.5). If a struct/enum name fails to compile,
 *      the fetched component header is the source of truth — the field
 *      semantics here are correct; only spellings may drift between versions.
 *   4. Orientation/colour: if the feed is upside-down/mirrored flip
 *      CAM_UVC_PPA_ROTATION / MIRROR_*; if colours are swapped flip
 *      CAM_UVC_JPEG_RGB_ORDER (RGB <-> BGR). A rear-view "mirror" feel usually
 *      wants CAM_UVC_PPA_MIRROR_X = true.
 * =====================================================================
 */

#include "camera.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "driver/ppa.h"
#include "driver/jpeg_decode.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lvgl_port.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"

static const char *TAG = "CAM_UVC";

/* ============================================================
 * CONFIGURATION
 * ============================================================ */

/* Kconfig defaults (defensive — this file only compiles under CAMERA_BACKEND_UVC) */
#ifndef CONFIG_CAMERA_UVC_H_RES
#define CONFIG_CAMERA_UVC_H_RES 1280
#endif
#ifndef CONFIG_CAMERA_UVC_V_RES
#define CONFIG_CAMERA_UVC_V_RES 720
#endif
#ifndef CONFIG_CAMERA_UVC_FPS
#define CONFIG_CAMERA_UVC_FPS 30
#endif
#ifndef CONFIG_CAMERA_UVC_VID
#define CONFIG_CAMERA_UVC_VID 0x0
#endif
#ifndef CONFIG_CAMERA_UVC_PID
#define CONFIG_CAMERA_UVC_PID 0x0
#endif

/* Decoded frame geometry — the webcam's MJPEG mode we request. */
#define SRC_H_RES CONFIG_CAMERA_UVC_H_RES
#define SRC_V_RES CONFIG_CAMERA_UVC_V_RES
#define SRC_BPP 2 /* RGB565 out of the JPEG decoder */
#define DEC_FB_SIZE (SRC_H_RES * SRC_V_RES * SRC_BPP)

/* Worst-case compressed MJPEG frame. C920 720p MJPEG is typically 60-150 KB;
 * cap generously at the uncompressed size so a rare low-compression frame
 * never overruns the input buffer. */
#define JPEG_IN_MAX (SRC_H_RES * SRC_V_RES)

/* Display frame buffer (panel native portrait; LVGL presents it rotated). */
#define DISP_H_RES 800
#define DISP_V_RES 1280
#define DISP_FB_SIZE (DISP_H_RES * DISP_V_RES * 2) /* RGB565 */

/* How long the processing task blocks waiting for a decoded frame before
 * looping to re-check s_running / mode transitions (so camera_stop() and a
 * reverse-gear release always take effect even if the camera stalls). */
#define CAM_FRAME_WAIT_MS 100

/* ---- PPA scale/rotate knobs (tune on the bench if orientation is off) ----
 * 90deg rotation maps the landscape webcam frame onto the portrait DPI buffer,
 * exactly like the CSI path. The scale factors below are specific to the
 * capture resolution (CONFIG_CAMERA_UVC_H_RES/V_RES) and fill the 800x1280
 * panel; see the per-macro note. */
#define CAM_UVC_PPA_ROTATION PPA_SRM_ROTATION_ANGLE_90
#define CAM_UVC_PPA_MIRROR_X false /* true = rear-view-mirror flip */
#define CAM_UVC_PPA_MIRROR_Y false
/* 640x480 source -> 800x1280 portrait: scale_x=2.0 (32/16) maps src width 640
 * -> output height 1280 (fills the panel); scale_y=1.625 (26/16) maps src
 * height 480 -> output width 780, centered in 800 (~10px bar each side). Both
 * land on the PPA 1/16 grid. (These are resolution-specific; re-derive if
 * CONFIG_CAMERA_UVC_H_RES/V_RES change.) */
#define CAM_UVC_PPA_SCALE_X (2.0f)    /* 32/16 */
#define CAM_UVC_PPA_SCALE_Y (1.625f)  /* 26/16 */

/* PiP: unrotated (LVGL sw-rotate orients the widget). 180 if it's upside-down
 * relative to fullscreen on the bench. */
#define CAM_UVC_PIP_ROTATION PPA_SRM_ROTATION_ANGLE_0

/* JPEG decode output colour order — flip if red/blue are swapped on screen. */
#define CAM_UVC_JPEG_RGB_ORDER JPEG_DEC_RGB_ELEMENT_ORDER_BGR

/* ============================================================
 * STATE
 * ============================================================ */
static ppa_client_handle_t s_ppa = NULL;
static SemaphoreHandle_t s_ppa_done = NULL;
static volatile bool s_running = false;
static TaskHandle_t s_task = NULL;
static SemaphoreHandle_t s_task_done = NULL;

/* USB / UVC / JPEG */
static bool s_usb_host_installed = false;
static bool s_uvc_installed = false;
static TaskHandle_t s_usb_lib_task = NULL;
static uvc_host_stream_hdl_t s_stream = NULL;
static jpeg_decoder_handle_t s_jpgd = NULL;
static uint8_t *s_jpeg_in = NULL;   /* DMA-capable MJPEG input scratch */
static size_t s_jpeg_in_cap = 0;
static uint8_t *s_dec_buf = NULL;   /* decoded RGB565 full frame (PPA source) */
static size_t s_dec_buf_cap = 0;
static QueueHandle_t s_frame_q = NULL; /* owned uvc_host_frame_t* from the driver */

/* PiP output target (set once via camera_config_pip before streaming) */
static void *s_pip_buf = NULL;
static uint32_t s_pip_w = 0;
static uint32_t s_pip_h = 0;
static uint32_t s_pip_blk_h = 0; /* sensor rows used (centered vertical crop) */
static uint32_t s_pip_off_y = 0; /* first row of the crop */
static float s_pip_scale = 0.0f;
static void (*s_pip_frame_cb)(void) = NULL;

/* Fullscreen requests (written by other tasks) and actual state (written only
 * by the processing task while it holds / releases the LVGL lock). */
static volatile bool s_fs_manual = false;
static volatile bool s_fs_reverse = false;
static volatile bool s_fs_active = false;

/* ============================================================
 * INTERNAL: USB host library event pump
 * ============================================================ */
static void usb_lib_task(void *arg)
{
    while (1)
    {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
        {
            usb_host_device_free_all();
        }
    }
}

/* ============================================================
 * INTERNAL: UVC callbacks (run in the UVC driver task context)
 * ============================================================ */

/* Frame delivered by the driver. Hand it to the processing task if a slot is
 * free (take ownership -> return false) else drop it (return true so the driver
 * reclaims the buffer immediately). Never block or do heavy work here. */
static bool uvc_frame_cb(const uvc_host_frame_t *frame, void *user_ctx)
{
    if (!s_running || !s_frame_q)
    {
        return true; /* let the driver reclaim */
    }
    uvc_host_frame_t *f = (uvc_host_frame_t *)frame;
    if (xQueueSend(s_frame_q, &f, 0) == pdTRUE)
    {
        return false; /* we own it now; returned later via uvc_host_frame_return */
    }
    return true; /* processing task still busy — drop this frame */
}

static void uvc_stream_cb(const uvc_host_stream_event_data_t *event, void *user_ctx)
{
    switch (event->type)
    {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGW(TAG, "UVC transfer error: 0x%x", event->transfer_error.error);
        break;
    case UVC_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "UVC camera disconnected");
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "UVC frame buffer overflow (increase frame_size)");
        break;
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        ESP_LOGW(TAG, "UVC frame underflow");
        break;
    default:
        break;
    }
}

/* ============================================================
 * INTERNAL: decode one MJPEG frame into s_dec_buf (RGB565)
 * ============================================================ */
static esp_err_t decode_frame(const uvc_host_frame_t *frame)
{
    size_t len = frame->data_len;
    if (len == 0 || len > s_jpeg_in_cap)
    {
        ESP_LOGW(TAG, "Bad MJPEG frame len=%u (cap=%u)", (unsigned)len, (unsigned)s_jpeg_in_cap);
        return ESP_ERR_INVALID_SIZE;
    }
    /* Copy into the DMA-capable input scratch (the driver's frame buffer is not
     * guaranteed to satisfy the JPEG engine's alignment). ~100 KB @ 30fps. */
    memcpy(s_jpeg_in, frame->data, len);

    jpeg_decode_cfg_t cfg = {
        .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
        .rgb_order = CAM_UVC_JPEG_RGB_ORDER,
        .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
    };
    uint32_t out_size = 0;
    esp_err_t ret = jpeg_decoder_process(s_jpgd, &cfg, s_jpeg_in, (uint32_t)len,
                                         s_dec_buf, (uint32_t)s_dec_buf_cap, &out_size);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "JPEG decode failed: 0x%x (%s)", ret, esp_err_to_name(ret));
    }
    return ret;
}

/* ============================================================
 * INTERNAL: processing task
 *   frame queue -> JPEG decode -> PPA scale/rotate into the active target
 * ============================================================ */
typedef struct
{
    void *disp_fb;
} cam_task_args_t;

static bool ppa_trans_done_cb(ppa_client_handle_t ppa_client,
                              ppa_event_data_t *edata, void *user_data)
{
    BaseType_t hp_task_woken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)user_data, &hp_task_woken);
    return hp_task_woken == pdTRUE;
}

static void camera_stream_task(void *arg)
{
    cam_task_args_t *ta = (cam_task_args_t *)arg;
    void *disp_fb = ta->disp_fb;
    free(ta);

    /* Output footprint after 90deg rotation (mirror of the CSI path). */
    uint32_t out_w = (uint32_t)(CAM_UVC_PPA_SCALE_Y * SRC_V_RES);
    uint32_t off_x = (DISP_H_RES > out_w) ? (DISP_H_RES - out_w) / 2 : 0;

    ppa_srm_oper_config_t srm_fs = {
        .in = {
            .buffer = NULL, /* set per frame to s_dec_buf */
            .pic_w = SRC_H_RES,
            .pic_h = SRC_V_RES,
            .block_w = SRC_H_RES,
            .block_h = SRC_V_RES,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer = disp_fb,
            .buffer_size = DISP_FB_SIZE,
            .pic_w = DISP_H_RES,
            .pic_h = DISP_V_RES,
            .block_offset_x = off_x,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = CAM_UVC_PPA_ROTATION,
        .scale_x = CAM_UVC_PPA_SCALE_X,
        .scale_y = CAM_UVC_PPA_SCALE_Y,
        .mirror_x = CAM_UVC_PPA_MIRROR_X,
        .mirror_y = CAM_UVC_PPA_MIRROR_Y,
        .mode = PPA_TRANS_MODE_BLOCKING,
        .user_data = s_ppa_done,
    };

    ppa_srm_oper_config_t srm_pip = {
        .in = {
            .buffer = NULL, /* set per frame to s_dec_buf */
            .pic_w = SRC_H_RES,
            .pic_h = SRC_V_RES,
            .block_w = SRC_H_RES,
            .block_h = s_pip_blk_h,
            .block_offset_x = 0,
            .block_offset_y = s_pip_off_y,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer = s_pip_buf,
            .buffer_size = s_pip_w * s_pip_h * 2,
            .pic_w = s_pip_w,
            .pic_h = s_pip_h,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = CAM_UVC_PIP_ROTATION,
        .scale_x = s_pip_scale,
        .scale_y = s_pip_scale,
        .mirror_x = CAM_UVC_PPA_MIRROR_X,
        .mirror_y = CAM_UVC_PPA_MIRROR_Y,
        .mode = PPA_TRANS_MODE_BLOCKING,
        .user_data = s_ppa_done,
    };

    const bool have_pip = (s_pip_buf != NULL);
    bool fs_active = false;

    ESP_LOGI(TAG, "UVC stream task up: src=%dx%d RGB565 -> fs (out_block_w=%u off_x=%u), pip=%ux%u",
             SRC_H_RES, SRC_V_RES, (unsigned)out_w, (unsigned)off_x,
             (unsigned)s_pip_w, (unsigned)s_pip_h);

    int frame_count = 0;
    while (s_running)
    {
        /* Apply fullscreen/PiP switches at frame boundaries — identical
         * lock-and-freeze contract to camera_csi.c. */
        bool want_fs = !have_pip || s_fs_manual || s_fs_reverse;
        if (want_fs && !fs_active)
        {
            lvgl_port_lock(portMAX_DELAY);
            memset(disp_fb, 0, DISP_FB_SIZE);
            esp_cache_msync(disp_fb, DISP_FB_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
            fs_active = true;
            s_fs_active = true;
            ESP_LOGI(TAG, "Camera fullscreen ON");
        }
        else if (!want_fs && fs_active)
        {
            lv_obj_invalidate(lv_screen_active());
            fs_active = false;
            s_fs_active = false;
            lvgl_port_unlock();
            ESP_LOGI(TAG, "Camera fullscreen OFF - PiP");
        }

        /* Wait for a frame the driver handed us (bounded so mode/stop are
         * re-checked even if the camera stalls). */
        uvc_host_frame_t *frame = NULL;
        if (xQueueReceive(s_frame_q, &frame, pdMS_TO_TICKS(CAM_FRAME_WAIT_MS)) != pdTRUE || !frame)
        {
            continue;
        }

        esp_err_t dret = decode_frame(frame);
        uvc_host_frame_return(s_stream, frame); /* give the buffer back ASAP */
        if (dret != ESP_OK)
        {
            continue;
        }

        ppa_srm_oper_config_t *op = fs_active ? &srm_fs : &srm_pip;
        op->in.buffer = s_dec_buf;
        esp_err_t pret = ppa_do_scale_rotate_mirror(s_ppa, op);
        if (pret != ESP_OK)
        {
            ESP_LOGW(TAG, "PPA SRM failed: 0x%x (%s)", pret, esp_err_to_name(pret));
            continue;
        }
        /* BLOCKING PPA returns only once the transfer is done; the done-callback
         * also gives s_ppa_done. Drain it so it can't leak across iterations. */
        xSemaphoreTake(s_ppa_done, 0);

        if (!fs_active && s_pip_frame_cb)
        {
            s_pip_frame_cb();
        }

        frame_count++;
        if (frame_count <= 5 || frame_count % 300 == 0)
        {
            ESP_LOGI(TAG, "Frame %d%s", frame_count, fs_active ? " (fullscreen)" : "");
        }
    }

    if (fs_active)
    {
        lv_obj_invalidate(lv_screen_active());
        s_fs_active = false;
        lvgl_port_unlock();
    }

    s_task = NULL;
    xSemaphoreGive(s_task_done);
    vTaskDelete(NULL);
}

/* ============================================================
 * PUBLIC API
 * ============================================================ */

esp_err_t camera_init(void)
{
    ESP_LOGI(TAG, "Initializing UVC camera (%dx%d MJPEG @ %d fps -> RGB565)",
             SRC_H_RES, SRC_V_RES, CONFIG_CAMERA_UVC_FPS);

    /* --- USB host library + event pump --- */
    const usb_host_config_t host_config = {
        .skip_phy_setup = false, /* P4 High-Speed OTG internal PHY */
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t ret = usb_host_install(&host_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "usb_host_install failed: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }
    s_usb_host_installed = true;
    xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, NULL, 5, &s_usb_lib_task, 0);

    /* --- UVC class driver --- */
    const uvc_host_driver_config_t uvc_cfg = {
        .driver_task_stack_size = 6 * 1024,
        .driver_task_priority = 6,
        .xCoreID = 0,
        .create_background_task = true,
    };
    ret = uvc_host_install(&uvc_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "uvc_host_install failed: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }
    s_uvc_installed = true;

    /* --- Hardware JPEG decoder + DMA-capable buffers --- */
    jpeg_decode_engine_cfg_t eng_cfg = {
        .intr_priority = 0,
        .timeout_ms = 40,
    };
    ret = jpeg_new_decoder_engine(&eng_cfg, &s_jpgd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "jpeg_new_decoder_engine failed: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    jpeg_decode_memory_alloc_cfg_t in_mem = {.buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER};
    s_jpeg_in = (uint8_t *)jpeg_alloc_decoder_mem(JPEG_IN_MAX, &in_mem, &s_jpeg_in_cap);
    jpeg_decode_memory_alloc_cfg_t out_mem = {.buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER};
    s_dec_buf = (uint8_t *)jpeg_alloc_decoder_mem(DEC_FB_SIZE, &out_mem, &s_dec_buf_cap);
    if (!s_jpeg_in || !s_dec_buf)
    {
        ESP_LOGE(TAG, "JPEG buffer alloc failed (in=%p/%u out=%p/%u)",
                 s_jpeg_in, (unsigned)s_jpeg_in_cap, s_dec_buf, (unsigned)s_dec_buf_cap);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "JPEG buffers: in=%u B, out=%u B", (unsigned)s_jpeg_in_cap, (unsigned)s_dec_buf_cap);

    /* --- PPA client for scale + rotate into the (RGB565) display buffer --- */
    s_ppa_done = xSemaphoreCreateBinary();
    s_frame_q = xQueueCreate(1, sizeof(uvc_host_frame_t *));
    if (!s_ppa_done || !s_frame_q)
    {
        ESP_LOGE(TAG, "Failed to create sync primitives");
        return ESP_ERR_NO_MEM;
    }
    ppa_client_config_t ppa_cfg = {
        .oper_type = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
    };
    ESP_ERROR_CHECK(ppa_register_client(&ppa_cfg, &s_ppa));
    ppa_event_callbacks_t ppa_cbs = {.on_trans_done = ppa_trans_done_cb};
    ESP_ERROR_CHECK(ppa_client_register_event_callbacks(s_ppa, &ppa_cbs));

    ESP_LOGI(TAG, "UVC camera ready (waiting for device on start)");
    return ESP_OK;
}

esp_err_t camera_config_pip(void *buf, uint32_t w, uint32_t h,
                            void (*frame_cb)(void))
{
    if (!buf || !frame_cb || w == 0 || h == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_running)
    {
        ESP_LOGE(TAG, "camera_config_pip must be called before camera_start");
        return ESP_ERR_INVALID_STATE;
    }
    /* Identical PPA 1/16-grid constraints to the CSI path, but against the UVC
     * source geometry (SRC_H_RES/SRC_V_RES). w must be a multiple of
     * SRC_H_RES/16 so the full-width scale lands exactly on the grid. */
    if ((w * 16) % SRC_H_RES != 0)
    {
        ESP_LOGE(TAG, "PiP width %u off the PPA 1/16 scale grid (use a multiple of %d)",
                 (unsigned)w, SRC_H_RES / 16);
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t n16 = (w * 16) / SRC_H_RES;
    uint32_t blk_h = (h * 16) / n16;
    if ((h * 16) % n16 != 0 || blk_h > SRC_V_RES)
    {
        ESP_LOGE(TAG, "PiP height %u needs a %u-px source crop at scale %u/16 (max %d)",
                 (unsigned)h, (unsigned)blk_h, (unsigned)n16, SRC_V_RES);
        return ESP_ERR_INVALID_ARG;
    }
    if (((uintptr_t)buf % 128) != 0 || (w * h * 2) % 128 != 0)
    {
        ESP_LOGE(TAG, "PiP buffer %p / size %u not 128-byte aligned",
                 buf, (unsigned)(w * h * 2));
        return ESP_ERR_INVALID_ARG;
    }

    s_pip_buf = buf;
    s_pip_w = w;
    s_pip_h = h;
    s_pip_blk_h = blk_h;
    s_pip_off_y = (SRC_V_RES - blk_h) / 2;
    s_pip_scale = (float)n16 / 16.0f;
    s_pip_frame_cb = frame_cb;

    ESP_LOGI(TAG, "PiP configured: %ux%u @ scale %u/16, source crop %dx%u+0+%u",
             (unsigned)w, (unsigned)h, (unsigned)n16,
             SRC_H_RES, (unsigned)blk_h, (unsigned)s_pip_off_y);
    return ESP_OK;
}

esp_err_t camera_start(esp_lcd_panel_handle_t panel)
{
    if (!s_uvc_installed)
    {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_running)
    {
        return ESP_OK;
    }

    void *disp_fb = NULL;
    esp_err_t fb_err = esp_lcd_dpi_panel_get_frame_buffer(panel, 1, &disp_fb);
    if (fb_err != ESP_OK || !disp_fb)
    {
        ESP_LOGE(TAG, "Failed to get DPI frame buffer (err=0x%x)", fb_err);
        return fb_err;
    }

    /* Open the stream — blocks up to the timeout for the camera to enumerate. */
    uvc_host_stream_config_t scfg = {
        .event_cb = uvc_stream_cb,
        .frame_cb = uvc_frame_cb,
        .user_ctx = NULL,
        .usb = {
            .vid = CONFIG_CAMERA_UVC_VID, /* 0 = first UVC device found */
            .pid = CONFIG_CAMERA_UVC_PID,
            .uvc_stream_index = 0,
        },
        .vs_format = {
            .h_res = SRC_H_RES,
            .v_res = SRC_V_RES,
            .fps = CONFIG_CAMERA_UVC_FPS,
            .format = UVC_VS_FORMAT_MJPEG,
        },
        .advanced = {
            .number_of_frame_buffers = 4,
            .frame_size = 0, /* 0 = driver estimates from the negotiated format */
            .frame_heap_caps = MALLOC_CAP_SPIRAM,
            /* Deep ISOC ring: at 720p30 only 4 URBs underruns during boot-time
             * contention (LVGL/network on core 0), dropping the EoF packet ->
             * "missed EoF" every frame. 8 URBs of 4x MPS ≈ 4 ms of buffering. */
            .number_of_urbs = 8,
            .urb_size = 0, /* 0 = driver picks 4x MPS */
        },
    };
    esp_err_t ret = uvc_host_stream_open(&scfg, pdMS_TO_TICKS(5000), &s_stream);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "uvc_host_stream_open failed: 0x%x (%s) — camera present? VBUS ok?",
                 ret, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "UVC stream open (%dx%d MJPEG @ %d fps)", SRC_H_RES, SRC_V_RES, CONFIG_CAMERA_UVC_FPS);

    s_running = true;

    if (!s_task_done)
    {
        s_task_done = xSemaphoreCreateBinary();
    }

    cam_task_args_t *ta = malloc(sizeof(cam_task_args_t));
    ta->disp_fb = disp_fb;
    /* Pin decode/PPA to core 1 so a long JPEG decode can't preempt the USB host
     * + UVC driver tasks (core 0) and stall the ISOC ring once frames flow. */
    xTaskCreatePinnedToCore(camera_stream_task, "cam_stream", 8192, ta, 6, &s_task, 1);

    ret = uvc_host_stream_start(s_stream);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "uvc_host_stream_start failed: 0x%x (%s)", ret, esp_err_to_name(ret));
        camera_stop();
        return ret;
    }

    return ESP_OK;
}

esp_err_t camera_stop(void)
{
    if (!s_running)
    {
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Stopping UVC stream");

    s_running = false;

    if (s_stream)
    {
        uvc_host_stream_stop(s_stream); /* no more frame callbacks after this */
    }

    /* Wait for the processing task to exit and release the LVGL lock. */
    if (s_task_done)
    {
        xSemaphoreTake(s_task_done, pdMS_TO_TICKS(2000));
    }

    /* Return any frame still queued before closing the stream. */
    if (s_frame_q)
    {
        uvc_host_frame_t *f = NULL;
        while (xQueueReceive(s_frame_q, &f, 0) == pdTRUE)
        {
            if (f && s_stream)
            {
                uvc_host_frame_return(s_stream, f);
            }
        }
    }
    if (s_stream)
    {
        uvc_host_stream_close(s_stream);
        s_stream = NULL;
    }

    if (s_task_done)
    {
        vSemaphoreDelete(s_task_done);
        s_task_done = NULL;
    }
    ESP_LOGI(TAG, "UVC stream stopped");
    return ESP_OK;
}

bool camera_is_running(void)
{
    return s_running;
}

void camera_set_reverse(bool in_reverse)
{
    s_fs_reverse = in_reverse;
}

void camera_set_fullscreen_manual(bool on)
{
    s_fs_manual = on;
}

bool camera_is_fullscreen(void)
{
    return s_fs_active;
}
