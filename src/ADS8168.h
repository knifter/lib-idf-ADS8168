#ifndef __ADS8168_H
#define __ADS8168_H

#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"


class ADS
{
    public:
        ADS(spi_host_device_t host, gpio_num_t cs);
        //~ADS();

        esp_err_t init();
        void setChannel(const uint8_t channelno);
        esp_err_t readChannel(uint16_t* counts, uint8_t* channel);

        esp_err_t read_test();
        esp_err_t acquire_bus();
        void release_bus();

    protected:
        typedef enum {
            ADCCMD_NOP         = 0b00000,
            ADCCMD_WR_REG      = 0b00001,
            ADCCMD_RD_REG      = 0b00010,
            ADCCMD_SET_BITS    = 0b00011,
            ADCCMD_CLR_BITS    = 0b00100
        } adc_cmd_t;

        typedef struct {
            uint8_t a, b;
        } pietje_t;
        
        void write_cmd(const adc_cmd_t cmd, const uint16_t address, const uint8_t data);

        spi_device_handle_t _spi;

    private:
        ADS(const ADS&);
        ADS& operator=(const ADS&);
};

#endif // __ADS8168_H