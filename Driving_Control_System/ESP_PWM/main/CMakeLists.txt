idf_component_register(SRCS "ESP_PWM.c"
                       PRIV_REQUIRES spi_flash "esp_timer" "esp_driver_pcnt" "esp_driver_mcpwm"
                       INCLUDE_DIRS ".")