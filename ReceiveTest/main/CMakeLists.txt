idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        display
        ui
        network
        camera
        esp_lvgl_port
        lvgl
)
