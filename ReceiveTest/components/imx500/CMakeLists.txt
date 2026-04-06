idf_component_register(SRCS "imx500.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES espressif__esp_cam_sensor esp_driver_i2c espressif__esp_sccb_intf)
