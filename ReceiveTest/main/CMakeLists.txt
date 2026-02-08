idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        display
        ui
        network
        esp_lvgl_port
        lvgl
)
