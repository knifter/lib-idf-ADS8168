#include "ADS8168.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/task.h"
#include <esp_log.h>
#include <esp_heap_caps.h>

#include "ADS8168_reg.h"


#include <math.h>
#include "HardwareSerial.h"
// #define LOG_LOCAL_LEVEL     ESP_LOG_VERBOSE
// #define DBG(msg, ...)      Serial.printf("%s: " msg "\n", __FUNCTION__, ##__VA_ARGS__)

// Helpers
#define CMD_ADDRESS_MASK    ((uint16_t)(1 << 11) - 1)

#define N_CHANNELS          4

ADS8168::ADS8168(spi_host_device_t host, gpio_num_t cs)
{
    spi_device_interface_config_t devcfg = 
    {
        command_bits: 0,
        address_bits: 0,
        dummy_bits: 0,
        mode: 0,
        duty_cycle_pos: 0,
        cs_ena_pretrans: 0,
        cs_ena_posttrans: 0,
        clock_speed_hz: SPI_MASTER_FREQ_40M,
        input_delay_ns: 20,
        spics_io_num: cs,
        flags: 0,
        queue_size: 256,
        pre_cb: NULL,
        post_cb: NULL
    };

    //Attach to the SPI bus
    esp_err_t ret = spi_bus_add_device(host, &devcfg, &_spi);
    ESP_ERROR_CHECK(ret);
}

esp_err_t ADS8168::init()
{
    acquire_bus();

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
    // write_cmd(ADCCMD_WR_REG, REG_DEVICE_CFG, DEVICE_CFG_SEQMODE_CUSTOM);
    // Manual channel seq moe
    write_cmd(ADCCMD_WR_REG, REG_DEVICE_CFG, DEVICE_CFG_SEQMODE_MANUAL);

    release_bus();

    return ESP_OK;
}

esp_err_t ADS8168::acquire_bus()
{
    if(_acquire_bus_cnt > 10)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = spi_device_acquire_bus(_spi, portMAX_DELAY);
    ESP_ERROR_CHECK(ret);
    if(ret == ESP_OK)
        _acquire_bus_cnt++;
    return ret;
}

void ADS8168::release_bus()
{
    if(!_acquire_bus_cnt)
        return;
    _acquire_bus_cnt--;
    if(_acquire_bus_cnt == 0)
        spi_device_release_bus(_spi);
}

void ADS8168::setChannel(const uint8_t channelno)
{
    acquire_bus();

    // select channel
    write_cmd(ADCCMD_WR_REG, REG_CHANNEL_ID, channelno & 0x07);
}

esp_err_t ADS8168::setSequence(const uint8_t length, const uint8_t* channels, const uint8_t repeat, const bool loop) 
{
    if(length<1 || length > 16)
        // ERROR("Sequence max length = 16");
        return ESP_ERR_INVALID_ARG;

    // Sequence mode enabled
    write_cmd(ADCCMD_WR_REG, REG_DEVICE_CFG, DEVICE_CFG_SEQMODE_CUSTOM);

    // Set channel sequence indexes
    for(uint8_t c=0; c<length; c++)
    {
        write_cmd(ADCCMD_WR_REG, REG_CCS_CHID_IDX(c), channels[c]);
        write_cmd(ADCCMD_WR_REG, REG_CCS_REPEAT_IDX(c), repeat);
    };

    // Repeat sequence, loop sequence
    write_cmd(ADCCMD_WR_REG, REG_CCS_SEQ_LOOP, loop ? CCS_SEQ_LOOP_EN : 0x00);

    // Sequence length, start-stop
    write_cmd(ADCCMD_WR_REG, REG_CCS_START_INDEX, 0);
    write_cmd(ADCCMD_WR_REG, REG_CCS_END_INDEX, length-1);


    return ESP_OK;
};

void ADS8168::sequenceStart()
{
    write_cmd(ADCCMD_WR_REG, REG_SEQ_START, SEQ_START_START);
}

uint16_t ADS8168::readChannel(uint8_t* channel_out)
{
    spi_transaction_t t;
    {
        // memset(&t, 0, sizeof(t));       //Zero out the transaction
        t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        // t.base.cmd = ;
        // t.base.addr = 0x0000;
        t.length = (channel_out != NULL) ? 24 : 16;
        t.rxlength = 0;
        t.tx_data[0] = 0x00;
        t.tx_data[1] = 0x00;
        t.tx_data[2] = 0x00;
        t.tx_data[3] = 0x00;
    };
    esp_err_t ret = spi_device_polling_transmit(_spi, &t);
    ESP_ERROR_CHECK(ret);

    if(channel_out != NULL)
        *channel_out = t.rx_data[2] >> 4;

    return t.rx_data[0] << 8 | t.rx_data[1];
}

void ADS8168::enableOTFMode()
{
    write_cmd(ADCCMD_WR_REG, REG_DEVICE_CFG, DEVICE_CFG_SEQMODE_ONTHEFLY);
    write_cmd(ADCCMD_WR_REG, REG_ON_THE_FLY_CFG, ON_THE_FLY_CFG_OTF_EN);
}

uint16_t ADS8168::readChannelOTF(const uint8_t otf_next_channel)
{    
    uint8_t cmd = ADCCMD_ONTHEFLY | (otf_next_channel & 0x03);

    // There appears to be a 'feature' in the adc where setting the otf channel repeatedly to
    // same one will return 0xFFFF.
    // static uint8_t last_otf = 0xFF;
    // if(last_otf == otf_next_channel)
    //     cmd = 0x00;
    // last_otf = otf_next_channel;

    spi_transaction_t t;
    {
        // memset(&t, 0, sizeof(t));       //Zero out the transaction
        t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        // t.base.cmd = ;
        // t.base.addr = 0x0000;
        t.length = 16;
        t.rxlength = 0;
        t.tx_data[0] = cmd << 3;
        t.tx_data[1] = 0x00;
        t.tx_data[2] = 0x00;
        t.tx_data[3] = 0x00;
    };
    esp_err_t ret = spi_device_polling_transmit(_spi, &t);
    ESP_ERROR_CHECK(ret);

    return t.rx_data[0] << 8 | t.rx_data[1];
}

void ADS8168::write_cmd(const adc_cmd_t cmd, const uint16_t address, const uint8_t data)
{
    spi_transaction_t t;
    {
        // memset(&t, 0, sizeof(t));       //Zero out the transaction
        t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        t.length = 24;
        t.rxlength = 0;
        t.tx_data[0] = cmd << 3;        // Top 3 address bits are discarded here!
        t.tx_data[1] = address;         // Top 3 address bits are discarded here!
        t.tx_data[2] = data;
    };
    // spi_transaction_t *rtrans;

    esp_err_t ret;
    // ret = spi_device_polling_start(_spi, &t, portMAX_DELAY);
    // ret = spi_device_queue_trans(_spi, &t, portMAX_DELAY);
    // ret = spi_device_polling_end(_spi, portMAX_DELAY);
    ret = spi_device_polling_transmit(_spi, &t);  //Transmit!
    ESP_ERROR_CHECK(ret);
}

