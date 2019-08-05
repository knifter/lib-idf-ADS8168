#ifndef __ADS8168_H
#define __ADS8168_H

class ADS
{
    public:
        ADS::ADS(spi_device_handle_t spi);
        //ADC::~ADS();

        esp_err_t init();

    protected:
        // Private methods
        void write_cmd(const adc_cmd_t cmd, const uint16_t address, const uint8_t data);

        // Variables
        spi_device_handle_t _spi;
    
    private:
        ADS(const ADS&);
        ADS& operator=(const ADS&);
};

#endif // __ADS8168_H