#ifndef __ADS8168_H
#define __ADS8168_H

#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

class ADS8168
{
    public:
        ADS8168(spi_host_device_t host, gpio_num_t cs);

        esp_err_t   init();
        esp_err_t   acquire_bus();
        void        release_bus();

        // Ad-Hoc mode
        void        setChannel(const uint8_t channelno);
        uint16_t    readChannel(uint8_t* channel_out = NULL);

        // Custom Sequence mode
        esp_err_t   setSequence(const uint8_t length, const uint8_t* channels, const uint8_t repeat = 1, bool loop = false);
        void        sequenceStart();

        // On-The-Fly Mode, Crappy 0xFFFF problems..
        void        enableOTFMode();
        uint16_t    readChannelOTF(const uint8_t otf_channel_next);

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
        ADS8168(const ADS8168&);
        ADS8168& operator=(const ADS8168&);
};

#endif // __ADS8168_H