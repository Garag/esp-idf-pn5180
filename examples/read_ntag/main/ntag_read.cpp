#include <stdio.h>
#include <stdlib.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pn5180.h"

static const char *TAG = "ntag_read";

extern "C" _Noreturn void app_main(void);

#define ESP32_HOST SPI2_HOST
#define PIN_NUM_MISO GPIO_NUM_13
#define PIN_NUM_MOSI GPIO_NUM_11
#define PIN_NUM_CLK  GPIO_NUM_12
#define PIN_NUM_NSS  GPIO_NUM_6
#define PIN_NUM_BUSY GPIO_NUM_5
#define PIN_NUM_RST  GPIO_NUM_7

pn5180 nfc(PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_NSS, PIN_NUM_BUSY, PIN_NUM_RST, ESP32_HOST);

_Noreturn void app_main()
{
    esp_err_t err;

    esp_log_level_set("esp-idf-pn5180", ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "ntag_read example");
    err = nfc.init();
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "PN5180 init failed!");
    }
    else {
        ESP_LOGI(TAG, "PN5180 init ok");
    }
    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
