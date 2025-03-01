
#pragma once

#include <driver/gpio.h>
#include <driver/spi_master.h>

// PN5180 Commands
#define PN5180_WRITE_REGISTER                     (0x00)
#define PN5180_WRITE_REGISTER_OR_MASK             (0x01)
#define PN5180_WRITE_REGISTER_AND_MASK            (0x02)
#define PN5180_WRITE_REGISTER_MULTIPLE            (0x03)
#define PN5180_READ_REGISTER                      (0x04)
#define PN5180_READ_REGISTER_MULTIPLE             (0x05)
#define PN5180_WRITE_EEPROM                       (0x06)
#define PN5180_READ_EEPROM                        (0x07)
#define PN5180_WRITE_TX_DATA                      (0x08)
#define PN5180_SEND_DATA                          (0x09)
#define PN5180_READ_DATA                          (0x0A)
#define PN5180_SWITCH_MODE                        (0x0B)
#define PN5180_MIFARE_AUTHENTICATE                (0x0C)
#define PN5180_EPC_INVENTORY                      (0x0D)
#define PN5180_EPC_RESUME_INVENTORY               (0x0E)
#define PN5180_EPC_RETRIEVE_INVENTORY_RESULT_SIZE (0x0F)
#define PN5180_EPC_RETRIEVE_INVENTORY_RESULT      (0x10)
#define PN5180_LOAD_RF_CONFIG                     (0x11)
#define PN5180_UPDATE_RF_CONFIG                   (0x12)
#define PN5180_RETRIEVE_RF_CONFIG_SIZE            (0x13)
#define PN5180_RETRIEVE_RF_CONFIG                 (0x14)
#define PN5180_RF_ON                              (0x16)
#define PN5180_RF_OFF                             (0x17)
#define PN5180_CONFIGURE_TESTBUS_DIGITAL          (0x18)
#define PN5180_CONFIGURE_TESTBUS_ANALOG           (0x19)

// PN5180 Registers
#define PN5180_SYSTEM_CONFIG        (0x00)
#define PN5180_IRQ_ENABLE           (0x01)
#define PN5180_IRQ_STATUS           (0x02)
#define PN5180_IRQ_CLEAR            (0x03)
#define PN5180_TRANSCEIVE_CONTROL   (0x04)
#define PN5180_TIMER1_RELOAD        (0x0C)
#define PN5180_TIMER1_CONFIG        (0x0F)
#define PN5180_RX_WAIT_CONFIG       (0x11)
#define PN5180_CRC_RX_CONFIG        (0x12)
#define PN5180_RX_STATUS            (0x13)
#define PN5180_TX_WAIT_CONFIG       (0x17)
#define PN5180_TX_CONFIG            (0x18)
#define PN5180_CRC_TX_CONFIG        (0x19)
#define PN5180_RF_STATUS            (0x1D)
#define PN5180_SYSTEM_STATUS        (0x24)
#define PN5180_TEMP_CONTROL         (0x25)
#define PN5180_AGC_REF_CONFIG	    (0x26)


// PN5180 EEPROM Addresses
#define PN5180_DIE_IDENTIFIER       (0x00)
#define PN5180_PRODUCT_VERSION      (0x10)
#define PN5180_FIRMWARE_VERSION     (0x12)
#define PN5180_EEPROM_VERSION       (0x14)
#define PN5180_IRQ_PIN_CONFIG       (0x1A)

// PN5180 IRQ_STATUS
#define PN5180_RX_IRQ_STAT         	    (1<<0)  // End of RF receiption IRQ
#define PN5180_TX_IRQ_STAT         	    (1<<1)  // End of RF transmission IRQ
#define PN5180_IDLE_IRQ_STAT       	    (1<<2)  // IDLE IRQ
#define PN5180_RFOFF_DET_IRQ_STAT  	    (1<<6)  // RF Field OFF detection IRQ
#define PN5180_RFON_DET_IRQ_STAT   	    (1<<7)  // RF Field ON detection IRQ
#define PN5180_TX_RFOFF_IRQ_STAT   	    (1<<8)  // RF Field OFF in PCD IRQ
#define PN5180_TX_RFON_IRQ_STAT    	    (1<<9)  // RF Field ON in PCD IRQ
#define PN5180_RX_SOF_DET_IRQ_STAT 	    (1<<14) // RF SOF Detection IRQ
#define PN5180_GENERAL_ERROR_IRQ_STAT   (1<<17) // General error IRQ
#define PN5180_LPCD_IRQ_STAT            (1<<19) // LPCD Detection IRQ

class pn5180 {
public:
    /**
     * Constructor needs pin settings
     * @param MISOpin SPI MISO pin
     * @param MOSIpin SPI MOSI pin
     * @param SCLKpin SPI clock pin
     * @param CSpin  SPI chip select pin
     * @param BUSYpin busy pin
     * @param RESETpin reset pin
     * @param spiHost SPI Host
     */
    pn5180(gpio_num_t MISOpin,
           gpio_num_t MOSIpin,
           gpio_num_t SCLKpin,
           gpio_num_t CSpin,
           gpio_num_t BUSYpin,
           gpio_num_t RESETpin,
           spi_host_device_t spiHost);

    /**
     * Initialize GPIO pins and SPI
     * @return ESP_OK if ok, otherwise failure
     */
    esp_err_t init();

    /**
     * Deinitialze resources
     */
    void deinit();

    /**
     * Reset PN5180 chip
     * @return ESP_OK if ok, otherwise failure
     */
    esp_err_t reset();

    /**
     * read PN5180 EEPROM
     * @param addr address
     * @param buffer buffer to receive data
     * @param len number of bytes to read
     * @return ESP_OK if ok, otherwise failure
     */
    esp_err_t readEEPROM(uint8_t addr, uint8_t *buffer, uint16_t len);

private:
    gpio_num_t MISOpin;
    gpio_num_t MOSIpin;
    gpio_num_t SCLKpin;
    gpio_num_t CSpin;
    gpio_num_t BUSYpin;
    gpio_num_t RESETpin;
    spi_host_device_t spiHost;

    spi_bus_config_t pn5180_buscfg;
    spi_device_interface_config_t pn5180_devcfg;
    spi_device_handle_t dev_handle;

    uint8_t ver_product[2]; // {minor ver, major ver}
    uint8_t ver_firmware[2]; // {minor ver, major ver}
    uint8_t ver_eeprom[2]; // {minor ver, major ver}

    /**
     * Send and receive data with respect to busy line
     * @param txBuffer pointer to transmit data
     * @param txLen number of bytes to transmit
     * @param rxBuffer pointer to receive buffer
     * @param rxLen number of bytes to receive
     * @return ESP_OK if ok, otherwise failure
     */
    esp_err_t transceiveCommand(uint8_t *txBuffer, size_t txLen, uint8_t *rxBuffer, size_t rxLen);

    /**
     * SPI transmit/receive data
     * @param dev SPI Host
     * @param tx tx buffer
     * @param txLen tx length
     * @param rx rx buffer
     * @param rxLen rx length
     * @return ESP_OK if ok, otherwise failure
     */
    esp_err_t spi_transmit(spi_device_handle_t dev, const void *tx, size_t txLen, void *rx, size_t rxLen);

    /**
     * The busy line indicates if the PN5180 is able to receive data.
     * Wait for busy line inactive (LOW) with a timeout.
     * @param timeout timeout in ticks
     * @return ESP_OK if ok, otherwise failure
     */
    esp_err_t waitReady(TickType_t timeout);
};
