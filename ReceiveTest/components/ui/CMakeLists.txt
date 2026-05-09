idf_component_register(
    SRCS
        "ui.c"
        "lv_font_montserrat_72.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        lvgl
        esp_lvgl_port
        can_spec
)

target_compile_definitions(__idf_ui PRIVATE LV_LVGL_H_INCLUDE_SIMPLE=1)
