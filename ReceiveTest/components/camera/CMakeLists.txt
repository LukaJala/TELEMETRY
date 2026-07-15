# The backup camera has two interchangeable backends behind camera.h. Exactly
# one .c is compiled, selected by CONFIG_CAMERA_BACKEND_* (see Kconfig ->
# "Backup camera").
#
# IMPORTANT: only SRCS may be gated on CONFIG_*. Component REQUIRES are expanded
# in an early pass *before* Kconfig is loaded, so a CONFIG-gated REQUIRES silently
# falls through to the wrong branch — the sources switch in the later pass but the
# include graph does not, and the selected backend's headers go missing. So
# REQUIRES lists the union of both backends' dependencies. The unused backend's
# components just link in dead (garbage-collected) — harmless.

if(CONFIG_CAMERA_BACKEND_UVC)
    set(cam_srcs "camera_uvc.c")
else()
    set(cam_srcs "camera_csi.c")
endif()

idf_component_register(
    SRCS
        ${cam_srcs}
    INCLUDE_DIRS
        "include"
    REQUIRES
        # shared output path (both backends)
        esp_driver_ppa
        esp_lcd
        esp_hw_support
        esp_mm
        esp_lvgl_port
        # CSI backend (camera_csi.c)
        esp_driver_cam
        esp_driver_isp
        esp_driver_i2c
        esp_driver_gpio
        espressif__esp_cam_sensor
        espressif__esp_sccb_intf
        # UVC backend (camera_uvc.c)
        usb
        espressif__usb_host_uvc
        esp_driver_jpeg
)
