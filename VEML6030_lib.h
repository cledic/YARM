/*

*/
 
#ifndef VEML6030_H
#define VEML6030_H
  
//  default address is (0x90 >> 1)
#define I2C_ADDR 0x48

// All registers are 16 bit
#define VEML6030_CONF_REG           0
#define VEML6030_THRS_HIGH_WIN      1
#define VEML6030_THRS_LOW_WIN       2
#define VEML6030_POWER_SAVE         3
#define VEML6030_HIGH_RES_OUT       4       
#define VEML6030_WHITE_CHN_OUT      5       
#define VEML6030_INT_STATUS         6

// Configuration Register 0x00 bit setting
#define VEML6030_ALS_SM(x)          (((x) & 0x3) << 11)
#define VEML6030_ALS_IT(x)          (((x) & 0xF) <<  6)
#define VEML6030_ALS_PERS(x)        (((x) & 0x3) <<  4)
#define VEML6030_ALS_INT_EN         (1 << 1)
#define VEML6030_ALS_SD             (1 << 0)

// Power Saving  Register 0x03 bit setting  
#define VEML6030_PSM(x)             (((x) & 0x3) << 1)
#define VEML6030_PSM_EN             (1 << 0)

// Interrupt Status Register 0x06 bit setting
#define VEML6030_INT_THRS_HIGH      (1 << 15)
#define VEML6030_INT_THRS_LOW       (1 << 14)

#endif
