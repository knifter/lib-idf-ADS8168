#ifndef __ADS8168_REG_H
#define __ADS8168_REG_H

#define REG_ACCESS                      0x00    // p44
    #define REG_ACCESS_BITS             0b10101010

#define REG_PD_CNTL                     0x04    // p45
    #define PD_CNTL_PD_REFby2           0x10
    #define PD_CNTL_PD_REF              0x08
    #define PD_CNTL_PD_REFBUF           0x04
    #define PD_CNTL_PD_ADC              0x02

#define REG_SDI_CNTL                    0x08    // p45
    #define SDI_CNTL_SPI00              0x00
    #define SDI_CNTL_SPI01              0x01
    #define SDI_CNTL_SPI10              0x02
    #define SDI_CNTL_SPI11              0x03

#define REG_SDO_CNTL1                   0x0C    // p46
    #define SDO_CNTL1_UCMODE            0x40
    #define SDO_CNTL1_RIGHT_ALIGN       0x20
    #define SDO_CNTL1_INTERLEAVE        0x10
    #define SDO_CNTL1_SDO_WIDTH1        0x04
    #define SDO_CNTL1_MODE_SPI          0x00
    #define SDO_CNTL1_MODE_RETIMER      0x03

#define REG_SDO_CNTL2                   0x0D    // p47
    #define REG_SDO_CNTL2_DDR           0x01

#define REG_SDO_CNTL3                   0x0E    // p47, Do not use, reserved

#define REG_SDO_CNTL4                   0x0F    // p47
    #define SDO_CNTL4_SEQSTS_CFG        0x01

#define REG_DATA_CNTL                   0x10    // p48
    #define DATA_CNTL_FORMAT_SAMPLE     0x00
    #define DATA_CNTL_FORMAT_CHID       0x10
    #define DATA_CNTL_FORMAT_CHIDSTAT   0x20
    #define DATA_CNTL_DATA_VAL          0x01

#define REG_PARITY_CNTL                 0x11    // p48
    #define PARITY_CNTL_PARITY_EN       0x04

#define REG_OFST_CAL                    0x18    // p49
    #define OFST_CAL_5V0                0x00
    #define OFST_CAL_4V5                0x01
    #define OFST_CAL_4V096              0x02
    #define OFST_CAL_3V3                0x03
    #define OFST_CAL_3V0                0x04
    #define OFST_CAL_2V5                0x05
    #define OFST_CAL_5V0A               0x06    // ?
    #define OFST_CAL_5V0B               0x07    // ? p49

#define REG_REF_MRG1                    0x19    // p50
#define REG_REF_MRG2                    0x1A    // p50
    #define REF_MRG2_EN_MARG            0x01

#define REG_REFby2_MRG                  0x1B    // p51
    #define REFby2_MRG_OFST_MASK        0x70
    #define REFby2_MRG_EN_MARG          0x01

#define REG_AIN_CFG                     0x24    // p52
    #define AIN_CH1CH0_SEPARATE         0x00
    #define AIN_CH1CH0_PAIR             0x40
    #define AIN_CH1CH0_PSEUDODIFF       0x80
    #define AIN_CH3CH2_SEPARATE         0x00
    #define AIN_CH3CH2_PAIR             0x10
    #define AIN_CH3CH2_PSEUDODIFF       0x20
    #define AIN_CH5CH4_SEPARATE         0x00
    #define AIN_CH5CH4_PAIR             0x04
    #define AIN_CH5CH4_PSEUDODIFF       0x08
    #define AIN_CH7CH6_SEPARATE         0x00
    #define AIN_CH7CH6_PAIR             0x01
    #define AIN_CH7CH6_PSEUDODIFF       0x02

#define REG_COM_CFG                     0x27    // p53
    #define COM_CFG_PSEUDO              0x01

#define REG_DEVICE_CFG                  0x1C
    #define DEVICE_CFG_ALERT_STATUS     0x08
    #define DEVICE_CFG_ERROR_STATUS     0x04
    #define DEVICE_CFG_SEQMODE_MANUAL   0x00
    #define DEVICE_CFG_SEQMODE_ONTHEFLY 0x01
    #define DEVICE_CFG_SEQMODE_AUTO     0x02
    #define DEVICE_CFG_SEQMODE_CUSTOM   0x03
    
#define REG_CHANNEL_ID                  0x1D
#define REG_SEQ_START                   0x1E    // p56, just write 0x01
    #define SEQ_START_START             0x01
#define REG_SEQ_STOP                    0x1F    // p56, just write 0x01
    #define SEQ_STOP_STOP               0x01
#define REG_ON_THE_FLY_CFG              0x2A
    #define ON_THE_FLY_CFG_OTF_EN       0x01
#define REG_AUTO_SEQ_CFG1               0x80
    #define AUTO_SEQ_CFG1_EN_AIN7       0x80
    #define AUTO_SEQ_CFG1_EN_AIN6       0x40
    #define AUTO_SEQ_CFG1_EN_AIN5       0x20
    #define AUTO_SEQ_CFG1_EN_AIN4       0x10
    #define AUTO_SEQ_CFG1_EN_AIN3       0x08
    #define AUTO_SEQ_CFG1_EN_AIN2       0x04
    #define AUTO_SEQ_CFG1_EN_AIN1       0x02
    #define AUTO_SEQ_CFG1_EN_AIN0       0x01

#define REG_AUTO_SEQ_CFG2               0x82
    #define AUTO_SEQ_CFG2_AUTO_REPEAT   0x01

#define REG_CCS_START_INDEX             0x88    // p59
#define REG_CCS_END_INDEX               0x89    // p59
#define REG_CCS_SEQ_LOOP                0x8A
    #define CCS_SEQ_LOOP_EN             0x01

#define REG_CCS_CHID_IDX(m)             (0x8C+2*m)  // p58,p60
#define REG_CCS_REPEAT_IDX(m)               (0x8D+2*m)  // p58,p60

#define REG_CHANID_0        0x8C    // next index at +2 (0x8E)
#define REG_CHANCNT_0       0x8D    // next at +2 (0x8F) etc..


#endif // __ADS8168_REG_H
