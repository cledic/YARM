/*
 * IncFile1.h
 *
 * Created: 24/02/2016 22:13:34
 *  Author: c.dicaprio
 */ 
#include <stdint.h>

#ifndef ADSL362_LIB_H_
#define ADSL362_LIB_H_

/** Register definitions
*/
#define ADXL362_REG_DEVD_AD				(0x00)
#define ADXL362_REG_DEVID_MST			(0x01)
#define ADXL362_REG_PARTID				(0x02)
#define ADXL362_REG_REVID				(0x03)
#define ADXL362_REG_XDATA				(0x08)
#define ADXL362_REG_YDATA				(0x09)
#define ADXL362_REG_ZDATA				(0x0A)
#define ADXL362_REG_STATUS				(0x0B)
#define ADXL362_REG_FIFO_ENTRIES_L		(0x0C)
#define ADXL362_REG_FIFO_ENTRIES_H		(0x0D)
#define ADXL362_REG_XDATA_L				(0x0E)
#define ADXL362_REG_XDATA_H				(0x0F)
#define ADXL362_REG_YDATA_L				(0x10)
#define ADXL362_REG_YDATA_H				(0x11)
#define ADXL362_REG_ZDATA_L				(0x12)
#define ADXL362_REG_ZDATA_H				(0x13)
#define ADXL362_REG_TEMP_L				(0x14)
#define ADXL362_REG_TEMP_H				(0x15)
#define ADXL362_REG_SOFT_RESET			(0x1F)
#define ADXL362_REG_THRESH_ACT_L		(0x20)
#define ADXL362_REG_THRESH_ACT_H		(0x21)
#define ADXL362_REG_TIME_ACT			(0x22)
#define ADXL362_REG_THRESH_INACT_L		(0x23)
#define ADXL362_REG_THRESH_INACT_H		(0x24)
#define ADXL362_REG_TIME_INACT_L		(0x25)
#define ADXL362_REG_TIME_INACT_H		(0x26)
#define ADXL362_REG_ACT_INACT_CTL		(0x27)
#define ADXL362_REG_FIFO_CONTROL		(0x28)
#define ADXL362_REG_FIFO_SAMPLES		(0x29)
#define ADXL362_REG_INTMAP1				(0x2A)
#define ADXL362_REG_INTMAP2				(0x2B)
#define ADXL362_REG_FILTER_CTL			(0x2C)
#define ADXL362_REG_POWER_CTL			(0x2D)
#define ADXL362_REG_SELF_TEST			(0x2E)

#define ADXL362_DEVD_AD					(0xAD)
#define ADXL362_DEVID_MST				(0x1D)
#define ADXL362_PARTID					(0xF2)

/* ADXL362_REG_STATUS definitions */
#define ADXL362_STATUS_ERR_USER_REGS        (1 << 7)
#define ADXL362_STATUS_AWAKE                (1 << 6)
#define ADXL362_STATUS_INACT                (1 << 5)
#define ADXL362_STATUS_ACT                  (1 << 4)
#define ADXL362_STATUS_FIFO_OVERRUN         (1 << 3)
#define ADXL362_STATUS_FIFO_WATERMARK       (1 << 2)
#define ADXL362_STATUS_FIFO_RDY             (1 << 1)
#define ADXL362_STATUS_DATA_RDY             (1 << 0)

/* ADXL362_REG_ACT_INACT_CTL definitions */
#define ADXL362_ACT_INACT_CTL_LINKLOOP(x)   (((x) & 0x3) << 4)
#define ADXL362_ACT_INACT_CTL_INACT_REF     (1 << 3)
#define ADXL362_ACT_INACT_CTL_INACT_EN      (1 << 2)
#define ADXL362_ACT_INACT_CTL_ACT_REF       (1 << 1)
#define ADXL362_ACT_INACT_CTL_ACT_EN        (1 << 0)

/* ADXL362_ACT_INACT_CTL_LINKLOOP(x) options */
#define ADXL362_MODE_DEFAULT        0
#define ADXL362_MODE_LINK           1
#define ADXL362_MODE_LOOP           3

/* ADXL362_REG_FIFO_CTL */
#define ADXL362_FIFO_CTL_AH                 (1 << 3)
#define ADXL362_FIFO_CTL_FIFO_TEMP          (1 << 2)
#define ADXL362_FIFO_CTL_FIFO_MODE(x)       (((x) & 0x3) << 0)

/* ADXL362_FIFO_CTL_FIFO_MODE(x) options */
#define ADXL362_FIFO_DISABLE              0
#define ADXL362_FIFO_OLDEST_SAVED         1
#define ADXL362_FIFO_STREAM               2
#define ADXL362_FIFO_TRIGGERED            3

/* ADXL362_REG_INTMAP1 */
#define ADXL362_INTMAP1_INT_LOW             (1 << 7)
#define ADXL362_INTMAP1_AWAKE               (1 << 6)
#define ADXL362_INTMAP1_INACT               (1 << 5)
#define ADXL362_INTMAP1_ACT                 (1 << 4)
#define ADXL362_INTMAP1_FIFO_OVERRUN        (1 << 3)
#define ADXL362_INTMAP1_FIFO_WATERMARK      (1 << 2)
#define ADXL362_INTMAP1_FIFO_READY          (1 << 1)
#define ADXL362_INTMAP1_DATA_READY          (1 << 0)

/* ADXL362_REG_INTMAP2 definitions */
#define ADXL362_INTMAP2_INT_LOW             (1 << 7)
#define ADXL362_INTMAP2_AWAKE               (1 << 6)
#define ADXL362_INTMAP2_INACT               (1 << 5)
#define ADXL362_INTMAP2_ACT                 (1 << 4)
#define ADXL362_INTMAP2_FIFO_OVERRUN        (1 << 3)
#define ADXL362_INTMAP2_FIFO_WATERMARK      (1 << 2)
#define ADXL362_INTMAP2_FIFO_READY          (1 << 1)
#define ADXL362_INTMAP2_DATA_READY          (1 << 0)

/* ADXL362_REG_FILTER_CTL definitions */
#define ADXL362_FILTER_CTL_RANGE(x)         (((x) & 0x3) << 6)
#define ADXL362_FILTER_CTL_RES              (1 << 5)
#define ADXL362_FILTER_CTL_HALF_BW          (1 << 4)
#define ADXL362_FILTER_CTL_EXT_SAMPLE       (1 << 3)
#define ADXL362_FILTER_CTL_ODR(x)           (((x) & 0x7) << 0)

/* ADXL362_FILTER_CTL_RANGE(x) options */
#define ADXL362_RANGE_2G                0 /* +/-2 g */
#define ADXL362_RANGE_4G                1 /* +/-4 g */
#define ADXL362_RANGE_8G                2 /* +/-8 g */

/* ADXL362_FILTER_CTL_ODR(x) options */
#define ADXL362_ODR_12_5_HZ             0 /* 12.5 Hz */
#define ADXL362_ODR_25_HZ               1 /* 25 Hz */
#define ADXL362_ODR_50_HZ               2 /* 50 Hz */
#define ADXL362_ODR_100_HZ              3 /* 100 Hz */
#define ADXL362_ODR_200_HZ              4 /* 200 Hz */
#define ADXL362_ODR_400_HZ              5 /* 400 Hz */

/* ADXL362_REG_POWER_CTL definitions */
#define ADXL362_POWER_CTL_RES               (1 << 7)
#define ADXL362_POWER_CTL_EXT_CLK           (1 << 6)
#define ADXL362_POWER_CTL_LOW_NOISE(x)      (((x) & 0x3) << 4)
#define ADXL362_POWER_CTL_WAKEUP            (1 << 3)
#define ADXL362_POWER_CTL_AUTOSLEEP         (1 << 2)
#define ADXL362_POWER_CTL_MEASURE(x)        (((x) & 0x3) << 0)

/* ADXL362_POWER_CTL_LOW_NOISE(x) options */
#define ADXL362_NOISE_MODE_NORMAL           0
#define ADXL362_NOISE_MODE_LOW              1
#define ADXL362_NOISE_MODE_ULTRALOW         2

/* ADXL362_POWER_CTL_MEASURE(x) options */
#define ADXL362_MEASURE_STANDBY         0
#define ADXL362_MEASURE_ON              2

/* ADXL362_REG_SELF_TEST */
#define ADXL362_SELF_TEST_ST            (1 << 0)

/* ADXL362 device information */
#define ADXL362_DEVICE_AD               0xAD
#define ADXL362_DEVICE_MST              0x1D
#define ADXL362_PART_ID                 0xF2

/* ADXL362 Reset settings */
#define ADXL362_RESET_KEY               0x52

int32_t ADXL362_Init( void);
int32_t ADXL362_SoftReset( void);
int32_t ADXL362_GetDeviceID( uint8_t*id);
int32_t ADXL362_GetMEMSID( uint8_t*id);
int32_t ADXL362_GetPartID( uint8_t*id);
int32_t ADXL362_GetRevID( uint8_t*id);
int32_t ADXL362_GetStatus( uint8_t*status);
int32_t ADXL362_GetFifoValue(uint8_t* pBuffer);
int32_t ADXL362_SetMeasureMode( void);
int32_t ADXL362_GetAccValues( float*x, float*y, float*z);
int32_t ADXL362_MotionSwitch( int32_t ac_thre, int32_t inac_thre, int32_t inac_tmr);

void ADXL362_SetPowerMode(uint8_t pwrMode);
void ADXL362_SetRange(uint8_t gRange);
void ADXL362_SetOutputRate(uint8_t outRate);
void ADXL362_GetXyz(int16_t* x, int16_t* y, int16_t* z);
void ADXL362_GetGxyz(float* x, float* y, float* z);
void ADXL362_GetGxyz2(float* x, float* y, float* z);

float ADXL362_ReadTemperature(void);
void ADXL362_FifoSetup(uint8_t  mode, uint16_t waterMarkLvl, uint8_t  enTempRead);
void ADXL362_SetupActivityDetection(uint8_t  refOrAbs, uint16_t threshold, uint8_t  time);
void ADXL362_SetupInactivityDetection(uint8_t  refOrAbs, uint16_t threshold, uint16_t time);

void ADXL361_GetActivityStatusPollingMode( void);
void ADXL361_GetActivityStatusInterruptMode( void);

#endif /* ADSL362_LIB_H_ */
