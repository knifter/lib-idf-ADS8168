#include "ADS8168.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/task.h"

#include "config.h"

// Helpers
#define CMD_ADDRESS_MASK     ((1 << 11) - 1)

typedef enum {
    NOP         = 0b00000,
    WR_REG      = 0b00001,
    RD_REG      = 0b00010,
    SET_BITS    = 0b00011,
    CLR_BITS    = 0b00100
} adc_cmd_t;

ADS::ADS(spi_device_handle_t spi) : _spi(spi)
{

}

esp_err_t ADS::init()
{
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
    tmp.p.cmdaddress = (cmd << 19) | ((address & CMD_ADDRESS_MASK) << 8);
    tmp.p.data = data;

    spi_transaction_t t;
    {
        memset(&t, 0, sizeof(t));       //Zero out the transaction
        t.length=24;                    //Command is 24 bits:
                                        // [cmd:5][address:11][data:8]
        t.tx_buffer = &(tmp.buf);             //The data is the cmd itself
        t.user=(void*)0;                //D/C needs to be set to 0
    };
    esp_err_t ret = spi_device_polling_transmit(_spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}


