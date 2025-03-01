#include <string.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "pn5180.h"

static const char TAG[] = "esp-idf-pn5180";

pn5180::pn5180(gpio_num_t MISOpin, gpio_num_t MOSIpin, gpio_num_t SCLKpin,
               gpio_num_t CSpin, gpio_num_t BUSYpin, gpio_num_t RESETpin, spi_host_device_t spiHost)
       :
       MISOpin(MISOpin),
       MOSIpin(MOSIpin),
       SCLKpin(SCLKpin),
       CSpin(CSpin),
       BUSYpin(BUSYpin),
       RESETpin(RESETpin),
       spiHost(spiHost),
       dev_handle(nullptr)
{
    pn5180_buscfg = {
            .mosi_io_num = MOSIpin,
            .miso_io_num = MISOpin,
            .sclk_io_num = SCLKpin,
            .quadwp_io_num = GPIO_NUM_NC,
            .quadhd_io_num = GPIO_NUM_NC,
            .data4_io_num = GPIO_NUM_NC,
            .data5_io_num = GPIO_NUM_NC,
            .data6_io_num = GPIO_NUM_NC,
            .data7_io_num = GPIO_NUM_NC,
            .data_io_default_level = false,
            .max_transfer_sz = 508,
            .flags = SPICOMMON_BUSFLAG_MISO|SPICOMMON_BUSFLAG_MOSI|SPICOMMON_BUSFLAG_SCLK|SPICOMMON_BUSFLAG_MASTER,
            .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
            .intr_flags = 0,

    };

    pn5180_devcfg = {
            .mode = 0,
            .clock_speed_hz = 1000000,
            .spics_io_num = CSpin,
            .queue_size = 1,
            .pre_cb = nullptr,
            .post_cb = nullptr,
    };
}

esp_err_t pn5180::init()
{
    gpio_config_t io_conf = {};
    esp_err_t err;

    if ((CSpin == GPIO_NUM_NC) || (RESETpin == GPIO_NUM_NC) || (BUSYpin == GPIO_NUM_NC)) {
        ESP_LOGE(TAG, "pn5180::init(): CS, RESET and BUSY pin definitions are needed!");
        return ESP_ERR_INVALID_ARG;
    }

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pin_bit_mask = (1 << CSpin) | (1 << RESETpin);
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1 << BUSYpin);
    gpio_config(&io_conf);

    ESP_LOGD(TAG, "pn5180::init(): initialize SPI ...");
    err = spi_bus_initialize(spiHost, &pn5180_buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        if (err == ESP_ERR_INVALID_STATE) {
            ESP_LOGI(TAG, "SPI host possibly already initialized, continuing ...");
        }
        else {
            ESP_LOGE(TAG, "failed to initialize SPI bus!");
            return err;
        }
    }
    err = spi_bus_add_device(spiHost, &pn5180_devcfg, &dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize SPI device!");
        dev_handle = nullptr;
        return err;
    }

    reset();

    // PN5180 Product version
    readEEPROM(PN5180_PRODUCT_VERSION, ver_product, 2);
    if(0xff == ver_product[1]){
        ESP_LOGE(TAG, "Initialization failed. Reset to restart.");
        while(1) vTaskDelay(portMAX_DELAY);
    }
    ESP_LOGI(TAG,"Product version: %d.%d",ver_product[1],ver_product[0]);


    // PN5180 Firmware version
    readEEPROM(PN5180_FIRMWARE_VERSION, ver_firmware, 2);
    ESP_LOGI(TAG,"Firmware version: %d.%d",ver_firmware[1],ver_firmware[0]);

    // PN5180 EEPROM version
    readEEPROM(PN5180_EEPROM_VERSION, ver_eeprom, 2);
    ESP_LOGI(TAG,"EEPROM version: %d.%d",ver_eeprom[1],ver_eeprom[0]);

    return ESP_OK;
}

void pn5180::deinit()
{
    if (dev_handle != nullptr) {
        spi_bus_remove_device(dev_handle);
        dev_handle = nullptr;
    }

    if ((CSpin != GPIO_NUM_NC) || (RESETpin != GPIO_NUM_NC) || (BUSYpin != GPIO_NUM_NC)) {
        gpio_config_t io_conf = {
                .pin_bit_mask = (1ull << CSpin) | (1ull << RESETpin) | (1ull << BUSYpin),
                .mode = GPIO_MODE_DISABLE,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE
        };

        gpio_config(&io_conf);
    }
}

esp_err_t pn5180::reset()
{
    uint32_t retries = 10;

    gpio_set_level(RESETpin, 0);
    vTaskDelay(1);
    gpio_set_level(RESETpin, 1);

    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

esp_err_t pn5180::readEEPROM(uint8_t addr, uint8_t *buffer, uint16_t len)
{
    if ((len == 0) || (buffer == nullptr) || (addr+len) > 255) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd[3] = { PN5180_READ_EEPROM, addr, (uint8_t)len};
    return transceiveCommand(cmd, 3, buffer, len);
}

esp_err_t pn5180::transceiveCommand(uint8_t *txBuffer, size_t txLen, uint8_t *rxBuffer, size_t rxLen)
{
    esp_err_t err;

    ESP_LOGD(TAG, "transceiveCommand(): transmit data, wait not busy ...");
    err = waitReady(pdTICKS_TO_MS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "transceiveCommand(): BUSY timeout!");
        return err;
    }

    ESP_LOGD(TAG, "transceiveCommand(): transmit data");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, txBuffer, txLen, ESP_LOG_DEBUG);
    err = spi_transmit(dev_handle, txBuffer, txLen, nullptr, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "transceiveCommand(): SPI transmit data failed!");
        reset();
        return err;
    }

    if (!rxBuffer || rxLen == 0)
        return ESP_OK;

    ESP_LOGD(TAG, "transceiveCommand(): receive data, wait not busy ...");
    err = waitReady(pdTICKS_TO_MS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "transceiveCommand(): BUSY timeout!");
        return err;
    }

    memset(rxBuffer, 0xFF, rxLen);
    err = spi_transmit(dev_handle, rxBuffer, rxLen, rxBuffer, rxLen);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "transceiveCommand(): SPI receive data failed!");
        reset();
        return err;
    }

    ESP_LOGD(TAG, "transceiveCommand(): received data:");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, rxBuffer, rxLen, ESP_LOG_DEBUG);

    return ESP_OK;
}

esp_err_t pn5180::spi_transmit(spi_device_handle_t dev, const void *tx, size_t txLen, void *rx, size_t rxLen)
{
    spi_transaction_t ta = {
            .length = txLen * 8,
            .rxlength = rxLen * 8,
            .tx_buffer = tx,
            .rx_buffer = rx,
    };

    return spi_device_transmit(dev_handle, &ta);
}

esp_err_t pn5180::waitReady(TickType_t timeout)
{
    TickType_t startTicks = xTaskGetTickCount();
    while (gpio_get_level(BUSYpin) && (xTaskGetTickCount() - startTicks) < timeout) {
        vTaskDelay(1);
    }

    if (gpio_get_level(BUSYpin)) {
        reset();
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}