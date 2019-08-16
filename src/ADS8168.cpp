#include "ADS8168.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/task.h"
#include <esp_log.h>
#include <esp_heap_alloc_caps.h>

#include "config.h"
#include "log.h"

#include "ADS8168_reg.h"

#define LOG_LOCAL_LEVEL     ESP_LOG_VERBOSE

// Helpers
#define CMD_ADDRESS_MASK    ((uint16_t)(1 << 11) - 1)

#define N_CHANNELS          4

ADS::ADS(spi_host_device_t host, gpio_num_t cs)
{
    spi_device_interface_config_t devcfg;
    {
        memset(&devcfg, 0, sizeof(devcfg));
        devcfg.mode = 0,                                //SPI mode 0
        devcfg.clock_speed_hz = SPI_MASTER_FREQ_20M; 
        devcfg.spics_io_num = cs;                       //CS pin
        devcfg.queue_size = 256;                          
        devcfg.command_bits = 5;
        devcfg.address_bits = 11;
        //devcfg.pre_cb = NULL;                           //Specify pre-transfer callback to handle D/C line
    };

    //Attach to the SPI bus
    esp_err_t ret = spi_bus_add_device(host, &devcfg, &_spi);
    ESP_ERROR_CHECK(ret);
}

esp_err_t ADS::init()
{
    gpio_set_level(PIN_ADC_ENABLE, 1);
    spi_device_acquire_bus(_spi, portMAX_DELAY);
    gpio_set_level(PIN_ADC_ENABLE, 0);
    gpio_set_level(PIN_ADC_ENABLE, 1);

    // write_cmd(ADCCMD_NOP, 0xFFFF, 0x01);
    // write_cmd(ADCCMD_WR_REG, 0xFFFF, 0x55);
    // write_cmd(ADCCMD_RD_REG, 0xFFFF, 0x55);
    // write_cmd(ADCCMD_SET_BITS, 0xFFFF, 0x55);
    // write_cmd(ADCCMD_CLR_BITS, 0x3333, 0x05);

    // enable writing
    write_cmd(ADCCMD_WR_REG, REG_ACCESS, REG_ACCESS_BITS);

    // Powerup all except the ref/2 buffer
    write_cmd(ADCCMD_WR_REG, REG_PD_CNTL, PD_CNTL_PD_REFby2);

    // Data type: ADC value + 4-bit channel id
    write_cmd(ADCCMD_WR_REG, REG_DATA_CNTL, DATA_CNTL_FORMAT_CHID);

    // Vref = 4V096
    write_cmd(ADCCMD_WR_REG, REG_OFST_CAL, OFST_CAL_4V096);

    // Custom channel seq mode
    write_cmd(ADCCMD_WR_REG, REG_DEVICE_CFG, DEVICE_CFG_SEQMODE_CUSTOM);

    // Only use channels 0, 2, 4, 6
    write_cmd(ADCCMD_WR_REG, REG_AUTO_SEQ_CFG1, 
          AUTO_SEQ_CFG1_EN_AIN0 
        | AUTO_SEQ_CFG1_EN_AIN2 
        | AUTO_SEQ_CFG1_EN_AIN4 
        | AUTO_SEQ_CFG1_EN_AIN6
        );
    
    // Repeat sequence, loop sequence
    //write_cmd(WR_REG, REG_AUTO_SEQ_CFG2, AUTO_SEQ_CFG2_AUTO_REPEAT);
    //write_cmd(WR_REG, REG_CCS_SEQ_LOOP, CCS_SEQ_LOOP_EN);

    // Set channel sequence indexes
    write_cmd(ADCCMD_WR_REG, REG_CCS_CHID_IDX(0), 0);
    write_cmd(ADCCMD_WR_REG, REG_CCS_CHID_IDX(1), 2);
    write_cmd(ADCCMD_WR_REG, REG_CCS_CHID_IDX(2), 4);
    write_cmd(ADCCMD_WR_REG, REG_CCS_CHID_IDX(3), 6);
    write_cmd(ADCCMD_WR_REG, REG_CCS_END_INDEX, 4);

    write_cmd(ADCCMD_WR_REG, REG_CCS_END_INDEX, 4);

    gpio_set_level(PIN_ADC_ENABLE, 0);
    gpio_set_level(PIN_ADC_ENABLE, 1);
    spi_device_release_bus(_spi);
    gpio_set_level(PIN_ADC_ENABLE, 0);

    return ESP_OK;
}

void ADS::write_cmd(const adc_cmd_t cmd, const uint16_t address, const uint8_t data)
{
    spi_transaction_t t;
    {
        // memset(&t, 0, sizeof(t));       //Zero out the transaction
        t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        t.cmd = cmd;
        t.addr = address;
        t.length = 8;
        t.rxlength = 0;
        t.user=(void*)0;                //D/C needs to be set to 0
        t.tx_data[0] = data;
    };
    // spi_transaction_t *rtrans;

    esp_err_t ret;
    // ret = spi_device_polling_start(_spi, &t, portMAX_DELAY);
    // ret = spi_device_queue_trans(_spi, &t, portMAX_DELAY);
    // ret = spi_device_polling_end(_spi, portMAX_DELAY);
    ret = spi_device_polling_transmit(_spi, &t);  //Transmit!

    assert(ret==ESP_OK);                //Should have had no issues.
}

esp_err_t ADS::read_test()
{
    // Send SEQ_START (resets the sequence) and then
    // Write n-channels * 3 NOP words to receive all enabled channels:
    //  <WR_REG, SEQ_START, 1>, n*3*<NOP> = 3 + 4x3 = 15 bytes
    #define TOTAL_BYTES     (3 + 3*N_CHANNELS)

    // since the NOP's are all zeroes, we'll write this as one command
    static void* rxbuf = heap_caps_calloc(TOTAL_BYTES, 1, MALLOC_CAP_DMA);
    static void* txbuf = heap_caps_calloc(TOTAL_BYTES, 1, MALLOC_CAP_DMA);
    spi_transaction_t t;
    {
        memset(&t, 0, sizeof(t));       //Zero out the transaction
        t.flags = 0;
        t.cmd = (uint16_t) ADCCMD_WR_REG;
        t.addr = REG_SEQ_START;
        t.length = (TOTAL_BYTES - 2) * 8;      // each command is 3 bytes, subtract cmd+addr of this one
        t.rxlength = 0;                       // equal to tx

        t.rx_buffer = rxbuf;                // read into rxbuf
        t.tx_buffer = txbuf;                // stat reading one further as we're writing (also 2 less)
    };
    gpio_set_level(PIN_ADC_ENABLE, 1);
    esp_err_t ret = spi_device_polling_transmit(_spi, &t);
    gpio_set_level(PIN_ADC_ENABLE, 0);
    assert( ret == ESP_OK );

    return ESP_OK;
}

