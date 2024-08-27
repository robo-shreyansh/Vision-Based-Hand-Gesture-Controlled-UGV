#include <rcl/rclc.h>
#include <rclc/executor.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <stdio.h>
#include <time.h>
#include <unistd.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"

rcl_subscription_t cmd_subscription;



void appMain (void* arguments){
    rcl_allocator_t alloc = rcl_get_default_allocator();
    rcl_support_t support;
    rcl_node_t car;

    rcl_support_init(&support, 0, NULL, &alloc);
    rcl_node_init_default(&car, "car", "", &alloc);


}