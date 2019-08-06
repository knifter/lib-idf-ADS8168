#ifndef __ADS8168_H
#define __ADS8168_H

#ifndef ADS8168_CLOCKSPEED
#define ADS8168_CLOCKSPEED      20E6
#endif

#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"


class ADS
{
    public:
        ADS(spi_host_device_t host, gpio_num_t cs);
        //~ADS();

        esp_err_t init();

    protected:
        typedef enum {
            ADCCMD_NOP         = 0b00000,
            ADCCMD_WR_REG      = 0b00001,
            ADCCMD_RD_REG      = 0b00010,
            ADCCMD_SET_BITS    = 0b00011,
            ADCCMD_CLR_BITS    = 0b00100
        } adc_cmd_t;

        // Private methods
        void write_cmd(const adc_cmd_t cmd, const uint16_t address, const uint8_t data);
        esp_err_t read_test();

        // Variables
        // gpio_num_t _cs;
        // spi_host_device_t _host;
        spi_device_handle_t _spi;
    
    private:
        ADS(const ADS&);
        ADS& operator=(const ADS&);
};

#endif // __ADS8168_H