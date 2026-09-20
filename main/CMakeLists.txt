idf_component_register(SRCS "blink_example_main.c" "jetson_link.c" "my_platform.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c esp_driver_twai esp_driver_uart esp_driver_ledc esp_driver_gpio esp_adc bluepad32 btstack)
