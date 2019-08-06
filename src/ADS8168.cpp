#include "ADS8168.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/task.h"

#include "config.h"

#include "ADS8168_reg.h"

// Helpers
#define CMD_ADDRESS_MASK     ((1 << 11) - 1)

ADS::ADS(spi_host_device_t host, gpio_num_t cs)
{
    spi_device_interface_config_t devcfg;
    {
        memset(&devcfg, 0, sizeof(devcfg));
        devcfg.mode = 0,                                //SPI mode 0
        devcfg.clock_speed_hz = ADS8168_CLOCKSPEED; 
        devcfg.spics_io_num = cs;                       //CS pin
        devcfg.queue_size = 1;                          
        //devcfg.pre_cb = NULL;                           //Specify pre-transfer callback to handle D/C line
    };

    //Attach to the SPI bus
    esp_err_t ret = spi_bus_add_device(host, &devcfg, &_spi);
    ESP_ERROR_CHECK(ret);
}

esp_err_t ADS::init()
{
    write_cmd(ADCCMD_NOP, 0xFFFF, 0x55);
    write_cmd(ADCCMD_WR_REG, 0xFFFF, 0x55);
    write_cmd(ADCCMD_RD_REG, 0xFFFF, 0x55);
    write_cmd(ADCCMD_SET_BITS, 0xFFFF, 0x55);
    write_cmd(ADCCMD_CLR_BITS, 0x3333, 0x55);

    return ESP_OK;
}

void ADS::write_cmd(const adc_cmd_t cmd, const uint16_t address, const uint8_t data)
{
    union {
        struct {
            uint16_t cmdaddress;
            uint8_t data;
        } p;
        uint8_t buf[3];
    } tmp;
    tmp.p.cmdaddress = (cmd << 11) | ((address & CMD_ADDRESS_MASK) << 8);
    tmp.p.data = data;

    spi_transaction_t t;
    {
        // memset(&t, 0, sizeof(t));       //Zero out the transaction
        t.flags = SPI_TRANS_USE_TXDATA;
        t.cmd = 0;
        t.addr = 0;
        t.length=24;                    //Command is 24 bits: [cmd:5][address:11][data:8]
        t.rxlength = 0;
        t.user=(void*)0;                //D/C needs to be set to 0

        t.tx_data[0] = 0x11;
        t.tx_data[1] = 0x33;
        t.tx_data[2] = 0x55;
        t.tx_data[3] = 0xFF;
        
        //memcpy(t.tx_data, tmp.buf, 3); // FIXME: directly compose the message here
        // t.tx_buffer = &(tmp.buf);       //The data is the cmd itself
    };
    esp_err_t ret = spi_device_polling_transmit(_spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

esp_err_t ADS::read_test()
{
    //get_id cmd
    write_cmd(ADCCMD_NOP, 0x0000, 0x00);

    spi_transaction_t t;
    {
        // memset(&t, 0, sizeof(t));       //Zero out the transaction
        t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        t.cmd = 0;
        t.addr = 0;
        t.length=24;                    //Command is 24 bits: [cmd:5][address:11][data:8]
        t.rxlength = 0;                 // 0==copy from length, Receive 24 bits: [CHID:4][SampleData:16] FIXME: only 20 bits
        t.user=(void*)0;                //D/C needs to be set to 0

        t.tx_buffer = 0;                // NOP, addr:0, data:0  (tx_buffer = tx_data)
    };
    memset(&t, 0, sizeof(t));
    t.length=8*3; // bits: 3 bytes
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1; // D/C

    esp_err_t ret = spi_device_polling_transmit(_spi, &t);
    assert( ret == ESP_OK );

    // uint32_t val = *(uint32_t*) &(t.rx_data);

    return ESP_OK;
}

