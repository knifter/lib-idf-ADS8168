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
        esp_err_t setSequence(const uint8_t length, const uint8_t* channels, const uint8_t repeat = 1, bool loop = false);
        void sequenceStart();
        uint16_t readChannel(uint8_t* channel_out = NULL);
        void enableOTFMode();
        uint16_t readChannelOTF(const uint8_t otf_channel_next);


        esp_err_t read_test();
        esp_err_t acquire_bus();
        void release_bus();

    protected:
        typedef enum {
            ADCCMD_NOP         = 0b00000,
            ADCCMD_WR_REG      = 0b00001,
            ADCCMD_RD_REG      = 0b00010,
            ADCCMD_SET_BITS    = 0b00011,
            ADCCMD_CLR_BITS    = 0b00100,
            ADCCMD_ONTHEFLY    = 0b10000
        } adc_cmd_t;

        void write_cmd(const adc_cmd_t cmd, const uint16_t address, const uint8_t data);

        spi_device_handle_t _spi;
        uint8_t _acquire_bus_cnt = 0;

    private:
        ADS(const ADS&);
        ADS& operator=(const ADS&);
};

#endif // __ADS8168_H