idf_component_register(SRCS "blink_example_main.c" "my_platform.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c esp_driver_twai esp_adc bluepad32 btstack)
