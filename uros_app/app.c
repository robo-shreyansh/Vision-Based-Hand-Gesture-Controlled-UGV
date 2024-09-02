#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <stdio.h>
#include <time.h>
#include <unistd.h>


#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#include "driver/ledc.h"


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define motor_gpio_pins (int[]) {} 
#define motor_channels (ledc_channel_t []) {LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4}

/*
TO DO:
. Decide motor pin nums
. Decide subscribers and publishers
. Write respective callback functions
. Setup executor 
*/


// Setting up the timer
ledc_timer_config_t timer_config;
timer_config.speed_mode = LEDC_HIGH_SPEED_MODE;
timer_config.duty_resolution = LEDC_TIMER_1_BIT;
timer_config.timer_num = LEDC_TIMER_1;
timer_config.freq_hz = 1000; //1000 khz.
timer_config.clk_cfg = LEDC_AUTO_CLK;
ledc_timer_config(&timer_config);

// Setting up channel
ledc_channel_config_t channel_config[4];
for (int i=0; i<4;i++){
    channel_config[i].gpio_num = motor_gpio_pins[i];
    channel_config[i].duty = 0;
    channel_config[i].channel = motor_channels[i];
    channel_config[i].speed_mode = LEDC_HIGH_SPEED_MODE;
    channel_config[i].hpoint = 0;
    channel_config[i].timer_sel = LEDC_TIMER_1;
    ledc_channel_config(channel_config[i]);
    }



rcl_subscription_t cmd_subscription;

// Subscription callback
void cmd_sub_callback(){
    continue;
}


// Main function
void appMain (void* arguments){
    rcl_allocator_t alloc = rcl_get_default_allocator();
    rcl_support_t support;
    rcl_node_t car;

    rcl_support_init(&support, 0, NULL, &alloc);
    rcl_node_init_default(&car, "esp-car", "", &alloc);
}