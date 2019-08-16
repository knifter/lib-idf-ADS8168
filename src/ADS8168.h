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
        esp_err_t read_test();

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
        
        // Private methods
        void write_cmd(const adc_cmd_t cmd, const uint16_t address, const uint8_t data);

        // Variables
        spi_device_handle_t _spi;
    
    private:
        ADS(const ADS&);
        ADS& operator=(const ADS&);
};

#endif // __ADS8168_H