#include <stdio.h>
#include <stdlib.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pn5180.h"

static const char *TAG = "ntag_read";

void app_main()
{
    ESP_LOGI(TAG, "ntag_read example");
    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
