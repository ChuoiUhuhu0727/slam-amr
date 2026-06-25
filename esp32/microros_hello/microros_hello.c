#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

void micro_ros_task(void *arg)
{
    while (1) {
        printf("Waiting for micro-ROS agent...\n");
        while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        printf("Agent found, initializing...\n");

        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;
        rcl_node_t node;
        rcl_publisher_t publisher;

        if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
            vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }
        if (rclc_node_init_default(&node, "esp32_node", "", &support) != RCL_RET_OK) {
            rclc_support_fini(&support);
            vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }
        if (rclc_publisher_init_default(&publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "hello_esp32") != RCL_RET_OK) {
            if (rcl_node_fini(&node) != RCL_RET_OK) {}
            rclc_support_fini(&support);
            vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }

        std_msgs__msg__String msg;
        char text[] = "Hello from ESP32!";
        msg.data.data = text;
        msg.data.size = strlen(text);
        msg.data.capacity = sizeof(text);

        while (rmw_uros_ping_agent(200, 1) == RMW_RET_OK) {
            if (rcl_publish(&publisher, &msg, NULL) != RCL_RET_OK) { break; }
            printf("Published: %s\n", text);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        printf("Agent lost, retrying...\n");
        if (rcl_publisher_fini(&publisher, &node) != RCL_RET_OK) {}
        if (rcl_node_fini(&node) != RCL_RET_OK) {}
        rclc_support_fini(&support);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static size_t uart_port = UART_NUM_0;

void app_main(void)
{
    rmw_uros_set_custom_transport(true, (void *)&uart_port,
        esp32_serial_open, esp32_serial_close,
        esp32_serial_write, esp32_serial_read);
    xTaskCreate(micro_ros_task, "micro_ros_task", 16000, NULL, 5, NULL);
}
