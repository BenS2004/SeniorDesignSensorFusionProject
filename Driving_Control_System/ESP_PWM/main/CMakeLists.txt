idf_component_register(SRCS "ESP_PWM.c"
                       PRIV_REQUIRES spi_flash "esp_timer" "mcpwm_prelude.h" "soc/clk_tree_defs.h" "esp_clk_tree.h"
                       INCLUDE_DIRS ".")