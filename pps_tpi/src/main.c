#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stdio.h"

#define LED_GPIO 2  // Pin donde está conectado el LED

void app_main(void) {
    char *task_name = pcTaskGetName(NULL);
    ESP_LOGI(task_name, "Starting LED blink task");

    while (1) {
        ESP_LOGI(task_name, "Starting LED blink task");

        gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

        // Enciende el LED
        gpio_set_level(LED_GPIO, 1);
        printf("Task %s: LED ON\n", task_name);
        vTaskDelay(pdMS_TO_TICKS(250));  // Espera 500 ms

        // Apaga el LED
        gpio_set_level(LED_GPIO, 0);
        printf("Task %s: LED OFF\n", task_name);
        vTaskDelay(pdMS_TO_TICKS(250));  // Espera 500 ms
    }
}