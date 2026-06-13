idf_component_register(
    SRCS "main.c"
    REQUIRES
        display
        ui
        network
        camera
        can_spec
        esp_lvgl_port
        lvgl
)
