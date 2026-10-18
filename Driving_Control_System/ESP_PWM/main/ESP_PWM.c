
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/mcpwm_prelude.h"




void app_main(void){

    // Initialize the ESP32 timer
    mcpwm_timer_handle_t timer = NULL;

    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT, // Choose your source clock here
        .resolution_hz = 100000,               // e.g., 100 kHz resolution (1 tick = 1 µs)
        .period_ticks = 100,                     // e.g., 100 ticks period -> 1 kHz frequency
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP, // Count up for PWM
        .intr_priority = 0,                        // Interrupt priority
        .flags = {0, 0, 0}                               // No special flags
    };

    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    //Initialize Operator
    mcpwm_oper_handle_t op = NULL;
    mcpwm_operator_config_t op_config = {
        .group_id = 0,
        .flags = {0, 0, 0},
        .intr_priority = 0
    };

    ESP_ERROR_CHECK(mcpwm_new_operator(&op_config, &op));

    // Initialize Comparator
    mcpwm_cmpr_handle_t comparator = NULL;
    /*struct extra_mcpwm_comparator_config_t compflags = {
        .update_cmp_on_tez = true, // Update comparator on timer event
        .update_cmp_on_tep = false,
        .update_cmp_on_sync = false,
    };*/
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = 1,
        .intr_priority = 0,
    };

    ESP_ERROR_CHECK(mcpwm_new_comparator(op, &comparator_config, &comparator));

    //Initialize PWM generator
    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = 18, // Set the GPIO number for PWM output
        .flags.invert_pwm = 0
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(op, &generator_config, &generator));

    // Set the PWM duty cycle to 50%
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 50)); 
    //Set comparator event
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, 
        MCPWM_GEN_ACTION_LOW)));
    
    //Set timer event
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator, 
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, 
        MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    
    // connect timer to operator
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(op, timer));
    // enable and start the timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    while(1){
        printf("PWM running...\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }

    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_FULL));
    ESP_ERROR_CHECK(mcpwm_timer_disable(timer));

    //register the generator to the system
    ESP_ERROR_CHECK(mcpwm_del_generator(generator));

    //Return comparator resources to the system
    ESP_ERROR_CHECK(mcpwm_del_comparator(comparator));

    //Return operator resources to the system
    ESP_ERROR_CHECK(mcpwm_del_operator(op));


    // Return timer resources to the system
    ESP_ERROR_CHECK(mcpwm_del_timer(timer));
}