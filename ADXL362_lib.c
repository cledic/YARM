/*
 * ADXL362_lib.c
 *
 * Created: 24/02/2016 10:09:08
 *  Author: c.dicaprio
 *
 */ 
#include <compiler.h>
#include <board.h>
#include <conf_board.h>
#include <port.h>
#include <asf.h>
#include <stdlib.h>

#include "ADXL362_lib.h"

// PA17 /CS
#define ADXL362_CS_PIN			      PIN_PA17
#define ADXL362_CS_ACTIVE			  false
#define ADXL362_CS_INACTIVE           !ADXL362_CS_ACTIVE
// PA10 /INT1
#define ADXL362_INT1_PIN              PIN_PA10
#define ADXL362_INT1_ACTIVE           false
#define ADXL362_INT1_INACTIVE         !ADXL362_INT1_ACTIVE
// PA11 /INT2
#define ADXL362_INT2_PIN              PIN_PA11
#define ADXL362_INT2_ACTIVE           false
#define ADXL362_INT2_INACTIVE         !ADXL362_INT2_ACTIVE

#define ADXL362_WR_CMD				  (0x0A)
#define ADXL362_RD_CMD				  (0x0B)
#define ADXL362_FIFO_CMD			  (0x0D)

struct spi_module		ADXL362_spi_master_instance;
struct spi_slave_inst	ADXL362_slave;
status_code_genare_t	ADSL362_spi_ret;
volatile bool			ADXL362_trans_complete_spi_master = false;
int8_t selected_gRange = ADXL362_RANGE_2G;

/* Private function */
void ADXL362_configure_spi_master_callbacks( void);
void ADXL362_configure_spi_master( void);
int32_t ADXL362_Read( uint8_t reg, uint8_t*rxb, int32_t len);
int32_t ADXL362_Write( uint8_t reg, uint8_t*txb, int32_t len);
int32_t ADXL362_ReadFifo( uint8_t*rxb, int32_t len);
/* ************************************************************************ */

static void ADXL362_callback_spi_master( struct spi_module *const module)
{
	ADXL362_trans_complete_spi_master = true;
}


void ADXL362_configure_spi_master_callbacks(void)
{
	spi_register_callback( &ADXL362_spi_master_instance, ADXL362_callback_spi_master, SPI_CALLBACK_BUFFER_TRANSCEIVED);
	spi_enable_callback( &ADXL362_spi_master_instance, SPI_CALLBACK_BUFFER_TRANSCEIVED);
}

#define ADXL362_SPI_MODULE              SERCOM3
#define ADXL362_SPI_SERCOM_MUX_SETTING  SPI_SIGNAL_MUX_SETTING_E
#define ADXL362_SPI_SERCOM_PINMUX_PAD0  PINMUX_PA16D_SERCOM3_PAD0
#define ADXL362_SPI_SERCOM_PINMUX_PAD1  PINMUX_UNUSED
#define ADXL362_SPI_SERCOM_PINMUX_PAD2  PINMUX_PA18D_SERCOM3_PAD2
#define ADXL362_SPI_SERCOM_PINMUX_PAD3  PINMUX_PA19D_SERCOM3_PAD3
#define ADXL362_SLAVE_SELECT_PIN		ADXL362_CS_PIN // EXT2_PIN_SPI_SS_0

void ADXL362_configure_spi_master(void)
{

	struct spi_config				config_spi_master;
	struct spi_slave_inst_config	slave_dev_config;
	
	/* Configure and initialize software device instance of peripheral slave */
	spi_slave_inst_get_config_defaults( &slave_dev_config);
	slave_dev_config.ss_pin = ADXL362_SLAVE_SELECT_PIN;
	spi_attach_slave( &ADXL362_slave, &slave_dev_config);
	
	/* Configure, initialize and enable SERCOM SPI module */
	spi_get_config_defaults( &config_spi_master);
	
	config_spi_master.mode_specific.master.baudrate = (1000*1000);
	config_spi_master.transfer_mode    = SPI_TRANSFER_MODE_0;
	config_spi_master.mux_setting = ADXL362_SPI_SERCOM_MUX_SETTING;
	
	/* Configure pad 0 for data in */
	config_spi_master.pinmux_pad0 = ADXL362_SPI_SERCOM_PINMUX_PAD0;	// PA16
	
	/* Configure pad 1 as unused */
	config_spi_master.pinmux_pad1 = PINMUX_UNUSED;
	
	/* Configure pad 2 for data out */
	config_spi_master.pinmux_pad2 = ADXL362_SPI_SERCOM_PINMUX_PAD2;	// PA18
	
	/* Configure pad 3 for SCK */
	config_spi_master.pinmux_pad3 = ADXL362_SPI_SERCOM_PINMUX_PAD3;	// PA19
	spi_init( &ADXL362_spi_master_instance, ADXL362_SPI_MODULE, &config_spi_master);

	spi_enable( &ADXL362_spi_master_instance);

}

int32_t ADXL362_Init( void)
{
	struct port_config pin_conf;
	uint8_t id;
		
	port_get_config_defaults(&pin_conf);

	/*
		PIN_PA17 /CS		Output
		PIN_PA10 /INT1		Input
		PIN_PA11 /INT2		Input
	*/
	
	/* Do not initialize the INT pins if you are configuring the External Interrupt */
#if 0
	/* Set pin PIN_PA10 as inputs /INT1 */
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(ADXL362_INT1_PIN, &pin_conf);

	/* Set pin PIN_PA11 as inputs /INT2 */
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(ADXL362_INT2_PIN, &pin_conf);
#endif

	/* Configure /CS as outputs, turn active */
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(ADXL362_CS_PIN, &pin_conf);
	port_pin_set_output_level(ADXL362_CS_PIN, ADXL362_CS_INACTIVE);

	ADXL362_configure_spi_master();
	ADXL362_configure_spi_master_callbacks();

	ADXL362_SoftReset();
	ADXL362_GetDeviceID( &id);
	
	//
	if ( id != ADXL362_DEVD_AD) {
		return 1;
	}
	
	id=0;
	ADXL362_GetMEMSID( &id);	// 0x1D
	id=0;
	ADXL362_GetPartID( &id);	// 0xF2
	id=0;
	ADXL362_GetRevID( &id);		// 0x01
	
	ADXL362_SetRange( ADXL362_RANGE_2G);
	return 0;
}

int32_t ADXL362_SoftReset( void)
{
	uint8_t id=ADXL362_RESET_KEY;
	
	ADXL362_Write( ADXL362_REG_SOFT_RESET, &id, 1);
	delay_ms(1);
	
	return 0;
}

int32_t ADXL362_GetDeviceID( uint8_t*id)
{
	ADXL362_Read( ADXL362_REG_DEVD_AD, id, 1);
	
	return 0;
}

int32_t ADXL362_GetMEMSID( uint8_t*id)
{
	ADXL362_Read( ADXL362_REG_DEVID_MST, id, 1);
	
	return 0;
}

int32_t ADXL362_GetPartID( uint8_t*id)
{
	ADXL362_Read( ADXL362_REG_PARTID, id, 1);
	
	return 0;
}

int32_t ADXL362_GetRevID( uint8_t*id)
{
	ADXL362_Read( ADXL362_REG_REVID, id, 1);
	
	return 0;
}

int32_t ADXL362_GetStatus( uint8_t*status)
{
	ADXL362_Read( ADXL362_REG_STATUS, status, 1);
	
	return 0;
}

int32_t ADXL362_SetMeasureMode( void)
{
	uint8_t buff[3];
	
	/* Activity threshold */
	//buff[0] = ADXL362_WR_CMD;
	//buff[1] = ADXL362_REG_POWER_CTL;
	buff[0] = 0x02;
	
	ADXL362_Write( ADXL362_REG_POWER_CTL, buff, 1);
	ADXL362_Read( ADXL362_REG_STATUS, &buff[0], 1);
	
	ADXL362_Read( ADXL362_REG_POWER_CTL, &buff[1], 1);
	
	return 0;
}

int32_t ADXL362_GetAccValues( float*x, float*y, float*z)
{
	uint8_t rxbuff[3];
	
	rxbuff[0] = 0x02;
	ADXL362_Write( ADXL362_REG_POWER_CTL, rxbuff, 1);
	ADXL362_Read( ADXL362_REG_POWER_CTL, &rxbuff[0], 1);
	ADXL362_Read( ADXL362_REG_STATUS, &rxbuff[0], 1);
	
	ADXL362_Read( ADXL362_REG_XDATA_L, &rxbuff[0], 1);
	ADXL362_Read( ADXL362_REG_XDATA_H, &rxbuff[1], 1);
	*x = (int16_t)(rxbuff[0]+(rxbuff[1]<<8));
	*x /= 1000;
	
	ADXL362_Read( ADXL362_REG_YDATA_L, &rxbuff[0], 1);
	ADXL362_Read( ADXL362_REG_YDATA_H, &rxbuff[1], 1);
	*y = (int16_t)(rxbuff[0]+(rxbuff[1]<<8));
	*y /= 1000;

	ADXL362_Read( ADXL362_REG_ZDATA_L, &rxbuff[0], 1);
	ADXL362_Read( ADXL362_REG_ZDATA_H, &rxbuff[1], 1);
	*z = (int16_t)(rxbuff[0]+(rxbuff[1]<<8));
	*z /= 1000;
	
//	*x=(float)rxbuff[0]/255;
//	*y=(float)rxbuff[1]/255;
//	*z=(float)rxbuff[2]/255;
	
	return 0;
}

/**
 * \brief Places the device into standby/measure mode.
 *
 * \param pwrMode - Power mode. Possible values:
 *                   ADXL362_MEASURE_STANDBY - standby mode.
 *                   ADXL362_MEASURE_ON      - measure mode.
 *
 * \return None.
 */
void ADXL362_SetPowerMode(uint8_t pwrMode)
{
    uint8_t oldPowerCtl = 0;
    uint8_t newPowerCtl = 0;

    ADXL362_Read( ADXL362_REG_POWER_CTL, &oldPowerCtl, 1);
    newPowerCtl = oldPowerCtl & ~ADXL362_POWER_CTL_MEASURE(0x3);
    newPowerCtl = newPowerCtl | (pwrMode * ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_ON));
    ADXL362_Write( ADXL362_REG_POWER_CTL, &newPowerCtl, 1);

}

/**
 * \brief Selects the measurement range.
 *
 * \param gRange - Range option. Possible values:
 *                  ADXL362_RANGE_2G  -  +-2 g
 *                  ADXL362_RANGE_4G  -  +-4 g
 *                  ADXL362_RANGE_8G  -  +-8 g
 *                           
 * \return None.
 */
void ADXL362_SetRange(uint8_t gRange)
{
    uint8_t oldFilterCtl = 0;
    uint8_t newFilterCtl = 0;

    ADXL362_Read( ADXL362_REG_FILTER_CTL, &oldFilterCtl, 1);
    newFilterCtl = oldFilterCtl & ~ADXL362_FILTER_CTL_RANGE(0x3);
    newFilterCtl = newFilterCtl | ADXL362_FILTER_CTL_RANGE(gRange);
    ADXL362_Write( ADXL362_REG_FILTER_CTL, &newFilterCtl, 1);

    selected_gRange = (1 << gRange) * 2;
}

/**
 * \brief Selects the Output Data Rate of the device.
 *
 * \param outRate - Output Data Rate option. Possible values:
 *                   ADXL362_ODR_12_5_HZ  -  12.5Hz
 *                   ADXL362_ODR_25_HZ    -  25Hz
 *                   ADXL362_ODR_50_HZ    -  50Hz
 *                   ADXL362_ODR_100_HZ   -  100Hz
 *                   ADXL362_ODR_200_HZ   -  200Hz
 *                   ADXL362_ODR_400_HZ   -  400Hz
 *
 * \return None.
 */
void ADXL362_SetOutputRate(uint8_t outRate)
{
    uint8_t oldFilterCtl = 0;
    uint8_t newFilterCtl = 0;

    ADXL362_Read( ADXL362_REG_FILTER_CTL, &oldFilterCtl, 1);
    newFilterCtl = oldFilterCtl & ~ADXL362_FILTER_CTL_ODR(0x7);
    newFilterCtl = newFilterCtl | ADXL362_FILTER_CTL_ODR(outRate);
    ADXL362_Write( ADXL362_REG_FILTER_CTL, &newFilterCtl, 1);
    
}

/**
 * \brief Reads the 3-axis raw data from the accelerometer.
 *
 * \param x - Stores the X-axis data(as two's complement).
 * \param y - Stores the Y-axis data(as two's complement).
 * \param z - Stores the Z-axis data(as two's complement).
 *
 * \return None.
 */
void ADXL362_GetXyz(int16_t* x, int16_t* y, int16_t* z)
{
    uint8_t xyzValues[6] = {0, 0, 0, 0, 0, 0};

    ADXL362_Read( ADXL362_REG_XDATA_L, xyzValues, 6);
    *x = (int16_t)((xyzValues[1] << 8) + xyzValues[0]);
    *y = (int16_t)((xyzValues[3] << 8) + xyzValues[2]);
    *z = (int16_t)((xyzValues[5] << 8) + xyzValues[4]);
}

/**
 * \brief Reads the 3-axis raw data from the accelerometer and converts it to g.
 *
 * \param x - Stores the X-axis data.
 * \param y - Stores the Y-axis data.
 * \param z - Stores the Z-axis data.
 *
 * \return None.
 */
void ADXL362_GetGxyz(float* x, float* y, float* z)
{
    uint8_t xyzValues[6] = {0, 0, 0, 0, 0, 0};

	xyzValues[0] = 0x02;
	ADXL362_Write( ADXL362_REG_POWER_CTL, xyzValues, 1);
	ADXL362_Read( ADXL362_REG_POWER_CTL, &xyzValues[0], 1);
	ADXL362_Read( ADXL362_REG_STATUS, &xyzValues[0], 1);

#if 1
    ADXL362_Read( ADXL362_REG_XDATA_L, xyzValues, 6);
    *x = (int16_t)((xyzValues[1] << 8) + xyzValues[0]);
    *x /= (1000 / (selected_gRange / 2));
    *y = (int16_t)((xyzValues[3] << 8) + xyzValues[2]);
    *y /= (1000 / (selected_gRange / 2));
    *z = (int16_t)((xyzValues[5] << 8) + xyzValues[4]);
    *z /= (1000 / (selected_gRange / 2));
#else
    ADXL362_Read( ADXL362_REG_XDATA_L, &xyzValues[0], 1);
	ADXL362_Read( ADXL362_REG_XDATA_H, &xyzValues[1], 1);
    *x = (int16_t)((xyzValues[1] << 8) + xyzValues[0]);
    *x /= (1000 / (selected_gRange / 2));
	
	ADXL362_Read( ADXL362_REG_YDATA_L, &xyzValues[0], 1);
	ADXL362_Read( ADXL362_REG_YDATA_L, &xyzValues[1], 1);
    *y = (int16_t)((xyzValues[1] << 8) + xyzValues[0]);
    *y /= (1000 / (selected_gRange / 2));
	
	ADXL362_Read( ADXL362_REG_ZDATA_L, &xyzValues[0], 1);
	ADXL362_Read( ADXL362_REG_ZDATA_L, &xyzValues[1], 1);
    *z = (int16_t)((xyzValues[1] << 8) + xyzValues[0]);
    *z /= (1000 / (selected_gRange / 2));
#endif

}

void ADXL362_GetGxyz2(float* x, float* y, float* z)
{
    uint8_t xyzValues[6] = {0, 0, 0, 0, 0, 0};

    ADXL362_Read( ADXL362_REG_XDATA_L, xyzValues, 6);
    *x = (int16_t)((xyzValues[1] << 8) + xyzValues[0]);
    *x /= (1000 / (selected_gRange / 2));
    *y = (int16_t)((xyzValues[3] << 8) + xyzValues[2]);
    *y /= (1000 / (selected_gRange / 2));
    *z = (int16_t)((xyzValues[5] << 8) + xyzValues[4]);
    *z /= (1000 / (selected_gRange / 2));
}

/**
 * \brief Reads the temperature of the device.
 *
 * \return tempCelsius - The value of the temperature(degrees Celsius).
 */
float ADXL362_ReadTemperature( void)
{
    uint8_t   rawTempData[2] = {0, 0};
    uint16_t  signedTemp     = 0;
    float     tempCelsius    = 0;

    ADXL362_Read(ADXL362_REG_TEMP_L, rawTempData, 2);
    signedTemp = (short)(rawTempData[1] << 8) + rawTempData[0];
    tempCelsius = (float)signedTemp * 0.065;
    
    return tempCelsius;
}

/**
 * \brief Configures the FIFO feature.
 *
 * \param mode         - Mode selection.
 *                       Example: ADXL362_FIFO_DISABLE      -  FIFO is disabled.
 *                                ADXL362_FIFO_OLDEST_SAVED -  Oldest saved mode.
 *                                ADXL362_FIFO_STREAM       -  Stream mode.
 *                                ADXL362_FIFO_TRIGGERED    -  Triggered mode.
 * \param waterMarkLvl - Specifies the number of samples to store in the FIFO.
 * \param enTempRead   - Store Temperature Data to FIFO.
 *                       Example: 1 - temperature data is stored in the FIFO
 *                                    together with x-, y- and x-axis data.
 *                                0 - temperature data is skipped.
 *
 * \return None.
 */
void ADXL362_FifoSetup(uint8_t  mode, uint16_t waterMarkLvl, uint8_t  enTempRead)
{
    uint8_t writeVal = 0;
	uint8_t tmp;

    writeVal = ADXL362_FIFO_CTL_FIFO_MODE(mode) | (enTempRead * ADXL362_FIFO_CTL_FIFO_TEMP);
	if ( waterMarkLvl > 255)
		 writeVal |= ADXL362_FIFO_CTL_AH;
	
	tmp=waterMarkLvl&0xFF;
    ADXL362_Write( ADXL362_REG_FIFO_CONTROL, &writeVal, 1);
    ADXL362_Write( ADXL362_REG_FIFO_SAMPLES, &tmp, 1);
}

/**
 * \brief Configures activity detection.
 *
 * \param refOrAbs  - Referenced/Absolute Activity Select.
 *                    Example: 0 - absolute mode.
 *                             1 - referenced mode.
 * \param threshold - 11-bit unsigned value that the adxl362 samples are
 *                    compared to.
 * \param time      - 8-bit value written to the activity timer register. The 
 *                    amount of time (in seconds) is: time / ODR, where ODR - is 
 *                    the output data rate.
 *
 * \return None.
 */
void ADXL362_SetupActivityDetection(uint8_t  refOrAbs, uint16_t threshold, uint8_t  time)
{
    uint8_t oldActInactReg = 0;
    uint8_t newActInactReg = 0;
	uint8_t tmp[2];
	
	/* INT1 mapped to activity status, active low, */
	oldActInactReg=ADXL362_INTMAP1_INT_LOW | ADXL362_INTMAP1_ACT;
	ADXL362_Write( ADXL362_REG_INTMAP1, &oldActInactReg, 1);
    /* Configure motion threshold and activity timer. */
	tmp[1] = (threshold >> 8) & 0x07;
	tmp[0] = threshold & 0xFF;
    ADXL362_Write( ADXL362_REG_THRESH_ACT_L, tmp, 2);
    ADXL362_Write( ADXL362_REG_TIME_ACT, &time, 1);
    /* Enable activity interrupt and select a referenced or absolute configuration. */
    ADXL362_Read( ADXL362_REG_ACT_INACT_CTL, &oldActInactReg, 1);
    newActInactReg = oldActInactReg & ~ADXL362_ACT_INACT_CTL_ACT_REF;
    newActInactReg |= ADXL362_ACT_INACT_CTL_ACT_EN | (refOrAbs * ADXL362_ACT_INACT_CTL_ACT_REF);
    ADXL362_Write( ADXL362_REG_ACT_INACT_CTL, &newActInactReg, 1);
}


/**
 * \brief Configures inactivity detection.
 *
 * \param refOrAbs  - Referenced/Absolute Inactivity Select.
 *                    Example: 0 - absolute mode.
 *                             1 - referenced mode.
 * \param threshold - 11-bit unsigned value that the adxl362 samples are
 *                    compared to.
 * \param time      - 16-bit value written to the inactivity timer register. The 
 *                    amount of time (in seconds) is: time / ODR, where ODR - is  
 *                    the output data rate.
 *
 * \return None.
 */
void ADXL362_SetupInactivityDetection(uint8_t  refOrAbs, uint16_t threshold, uint16_t time)
{
    uint8_t oldActInactReg = 0;
    uint8_t newActInactReg = 0;
	uint8_t tmp[2];
    
	/* INT2 mapped to activity status, active low, */
	oldActInactReg=ADXL362_INTMAP2_INT_LOW | ADXL362_INTMAP2_INACT;
	ADXL362_Write( ADXL362_REG_INTMAP2, &oldActInactReg, 1);
    /* Configure motion threshold and inactivity timer. */
	tmp[1] = (threshold >> 8) & 0x07;
	tmp[0] = threshold & 0xFF;
    ADXL362_Write( ADXL362_REG_THRESH_INACT_L, tmp, 2);
	tmp[1] = (time >> 8);
	tmp[0] = time & 0xFF;	
    ADXL362_Write( ADXL362_REG_TIME_INACT_L, tmp, 2);
    /* Enable inactivity interrupt and select a referenced or absolute configuration. */
    ADXL362_Read( ADXL362_REG_ACT_INACT_CTL, &oldActInactReg, 1);
    newActInactReg = oldActInactReg & ~ADXL362_ACT_INACT_CTL_INACT_REF;
    newActInactReg |= ADXL362_ACT_INACT_CTL_INACT_EN | (refOrAbs * ADXL362_ACT_INACT_CTL_INACT_REF);
    ADXL362_Write( ADXL362_REG_ACT_INACT_CTL,  &newActInactReg, 1);
}

void ADXL361_GetActivityStatusInterruptMode( void)
{
	uint8_t regVal;

	ADXL362_SetPowerMode( 0);
	ADXL362_SetOutputRate( ADXL362_ODR_100_HZ);
	
	regVal = ADXL362_ACT_INACT_CTL_LINKLOOP(ADXL362_MODE_LINK);
	ADXL362_Write( ADXL362_REG_ACT_INACT_CTL, &regVal, 1);
	ADXL362_SetupActivityDetection(1, 60, 4);
	ADXL362_SetupInactivityDetection(1, 700, 250);
	ADXL362_SetPowerMode(1);
	/*!< Clear ACT and INACT bits by reading the Status Register. */
	ADXL362_Read( ADXL362_REG_STATUS, &regVal, 1);

}

void ADXL361_GetActivityStatusPollingMode( void)
{
	uint8_t regVal;
	uint8_t detections = 0;

	ADXL362_SetPowerMode( 0);
	ADXL362_SetOutputRate( ADXL362_ODR_100_HZ);
	
	regVal = ADXL362_ACT_INACT_CTL_LINKLOOP(ADXL362_MODE_LINK);
	ADXL362_Write( ADXL362_REG_ACT_INACT_CTL, &regVal, 1);
	ADXL362_SetupActivityDetection(1, 60, 4);
	ADXL362_SetupInactivityDetection(1, 700, 250);
	ADXL362_SetPowerMode(1);
	/*!< Clear ACT and INACT bits by reading the Status Register. */
	ADXL362_Read( ADXL362_REG_STATUS, &regVal, 1);
	/*!< Exit polling after 5 detections. */
	detections = 25;
	while(detections)
	{
		do /*!< Wait for the detection of an activity or inactivity. */
		{
			ADXL362_Read( ADXL362_REG_STATUS, &regVal, 1);
		}while(!(regVal & ADXL362_STATUS_ACT) && !(regVal & ADXL362_STATUS_INACT));
		detections--;
		if(regVal & ADXL362_STATUS_ACT)
		{
			printf("Activity\r\n");
		}
		if(regVal & ADXL362_STATUS_INACT)
		{
			printf("Inactivity\r\n");
		}
	}
	printf("Finished activity polling.\r\n");
}

void ADXL361_GetActivityStatusInterruptFifoMode( void)
{
	uint8_t regVal;

	ADXL362_SetPowerMode( 0);
	ADXL362_SetOutputRate( ADXL362_ODR_100_HZ);

#if 0
	/* Set FIFO samples and FIFO Triggered mode */
	regVal = 128;
	ADXL362_Write( ADXL362_REG_FIFO_SAMPLES, &regVal, 1);
	regVal = ADXL362_FIFO_CTL_FIFO_MODE(ADXL362_FIFO_TRIGGERED);
	ADXL362_Write( ADXL362_REG_FIFO_CONTROL, &regVal, 1);
#endif
	
	ADXL362_FifoSetup( ADXL362_FIFO_TRIGGERED, 510, 0);
	
	regVal = ADXL362_ACT_INACT_CTL_LINKLOOP(ADXL362_MODE_LINK);
	ADXL362_Write( ADXL362_REG_ACT_INACT_CTL, &regVal, 1);
	ADXL362_SetupActivityDetection(1, 60, 1 /*4*/);
	ADXL362_SetupInactivityDetection(1, 700, 1 /*250*/);
	ADXL362_SetPowerMode(1);
	/*!< Clear ACT and INACT bits by reading the Status Register. */
	ADXL362_Read( ADXL362_REG_STATUS, &regVal, 1);

}

/**
 * @brief   Riceve l'array in uscita dalla FIFO e ritorna i valori di X, Y e Z.
 *
 * @param   b   puntatore all'array dalla FIFO
 * @param   x   puntatore all'array per i valori X
 * @param   y   puntatore all'array per i valori Y
 * @param   z   puntatore all'array per i valori Z
 * @param   len numero di triplette X, Y e Z
 *
 * Formato dati dalla FIFO:
 *        11
 * |GG|SS|109876543210|
 *  GG
 *  00  X sample
 *  01  Y sample
 *  10  Z sample
  * SS  Sign extension
 */
uint32_t ADXL362_FifoBufferToXYZ( uint8_t*b, int16_t*x, int16_t*y, int16_t*z, uint32_t len)
{
    uint32_t i, v;
    uint8_t tmp;

    if ( len>512 || len==0)
        return 1;

    /* Allinea il primo byte al valore della X */
    i=0;
    while( (b[i+1] & 0xC0) != 0x00) {
        i+=2;
    };
    /* Se è avvenuto l'allineamento, tolgo un elemento dalla conversione. */
    if ( i)
        len--;

    v=0;
    while( i<(len*2) ) {
		switch( b[i+1] & 0xC0) {
			case 0x00:		// X sample
				if ( b[i+1]&0x30)		// Verifico i bit di sign e lo estendo
					tmp=b[i+1] | 0xC0;
				else
					tmp=b[i+1] & 0x3F;
				x[v] = (int16_t)((tmp<<8) | b[i]);
			break;
			case 0x40:		// Y sample
				if ( b[i+1]&0x30)		// Verifico i bit di sign e lo estendo
					tmp=b[i+1] | 0xC0;
				else
					tmp=b[i+1] & 0x3F;
				y[v] = (int16_t)((tmp<<8) | b[i]);
			break;
			case 0x80:		// Z sample
				if ( b[i+1]&0x30)		// Verifico i bit di sign e lo estendo
					tmp=b[i+1] | 0xC0;
				else
					tmp=b[i+1] & 0x3F;
				z[v] = (int16_t)((tmp<<8) | b[i]);
				v++;
			break;
		}
		//
		i+=2;
	}
	//
	return 0;
}

/*
 * 1. Write 250 decimal (0xFA) to Register 0x20, and write 0 to Register 0x21: sets activity threshold to 250 mg.
 * 2. Write 150 decimal (0x96) to Register 0x23, and write 0 to Register 0x24: sets inactivity threshold to 150 mg.
 * 3. Write 30 decimal (0x1E) to Register 0x25: sets inactivity timer to 30 samples or about 5 seconds.
 * 4. Write 0x3F to Register 0x27: configures motion detection in loop mode and enables referenced activity and inactivity detection.
 * 5. Write 0x40 to Register 0x2B: map the AWAKE bit to INT2. The INT2 pin is tied to the gate of the switch.
 * 6. Write 0x0A to Register 0x2D: begins the measurement in wake-up mode.
*/

int32_t ADXL362_MotionSwitch( int32_t ac_thre, int32_t inac_thre, int32_t inac_tmr)
{
	uint8_t buff[6];
	
	/* Activity threshold */
	//buff[0] = ADXL362_WR_CMD;
	//buff[1] = ADXL362_REG_THRESH_ACT_L;
	buff[0] = (ac_thre & 0xFF);				// lower reg
	buff[1] = (ac_thre & 0x700)>>8;			// 3 bits upper reg
	
	ADXL362_Write( ADXL362_REG_THRESH_ACT_L, buff, 2);
	
	/* Inactivity threshold and inactivity timer */
	//buff[0] = ADXL362_WR_CMD;
	//buff[1] = ADXL362_REG_THRESH_INACT_L;
	buff[0] = (inac_thre & 0xFF);				// lower reg
	buff[1] = (inac_thre & 0x700)>>8;			// next 3 bits upper reg
	buff[2] = (inac_tmr & 0xFF);				// next lower reg ADXL362_TIME_INACT_L
	buff[3] = (inac_tmr & 0xFF00)>>8;			// next upper reg ADXL362_TIME_INACT_H
	
	ADXL362_Write( ADXL362_REG_THRESH_INACT_L, buff, 4);

	/* Motion detection in loop */
	//buff[0] = ADXL362_WR_CMD;
	//buff[1] = ADXL362_REG_ACT_INACT_CTL;
	buff[0] = 0x3F;
	
	ADXL362_Write( ADXL362_REG_ACT_INACT_CTL, buff, 1);
	
	/* Event AWAKE on INT2 */
	//buff[0] = ADXL362_WR_CMD;
	//buff[1] = ADXL362_REG_INTMAP2;
	buff[0] = ADXL362_INTMAP2_INT_LOW | ADXL362_INTMAP2_AWAKE; // | ADXL362_INTMAP2_INACT | ADXL362_INTMAP2_ACT;  // 0x40;
	
	ADXL362_Write( ADXL362_REG_INTMAP2, buff, 1);

	/* Wake up mode */
	//buff[0] = ADXL362_WR_CMD;
	//buff[1] = ADXL362_REG_POWER_CTL;
	buff[0] = 0x0A;
	
	ADXL362_Write( ADXL362_REG_POWER_CTL, buff, 1);
	
	return 0;
}

/**
 * \brief Reads multiple bytes from the device's FIFO buffer.
 *
 * \param pBuffer  Stores the read bytes.
 *
 * \return         Number of FIFO byte read.
 */
int32_t ADXL362_GetFifoValue(uint8_t* pBuffer)
{
//    uint8_t  buffer[512+1];
    uint8_t  tmp[2];
    uint16_t len;

    ADXL362_Read( ADXL362_REG_FIFO_ENTRIES_L, tmp, 2);
    len = tmp[0]+(tmp[1]<<8);
    
	if ( len==0)
		return 0;
		
	if ( len>512)
		len=512;
		
    ADXL362_ReadFifo( pBuffer, len);

    //
//    for(index = 0; index < len; index++) {
//        pBuffer[index] = buffer[index + 1];
//    }
    
    return len;
}

int32_t ADXL362_Read( uint8_t reg, uint8_t*rxb, int32_t len)
{
	uint8_t ptr[34];
	uint8_t ptr_tx[34];
	uint8_t i;

	if ( len > 32)
	return 1;
	
	// buffer used to transmit CMD and Register
	ptr_tx[0] = ADXL362_RD_CMD;
	ptr_tx[1] = reg;
	
	//
	spi_select_slave(&ADXL362_spi_master_instance, &ADXL362_slave, true);
	
	ADSL362_spi_ret = spi_transceive_buffer_job(&ADXL362_spi_master_instance, ptr_tx, ptr, len+2);
	
	if ( ADSL362_spi_ret != 0) {
		spi_select_slave(&ADXL362_spi_master_instance, &ADXL362_slave, false);
		return ADSL362_spi_ret;
	}
	
	while (!ADXL362_trans_complete_spi_master) {
		/////* Wait for write and read complete */
	}
	ADXL362_trans_complete_spi_master = false;

	spi_select_slave(&ADXL362_spi_master_instance, &ADXL362_slave, false);
	
	// copy the returned data to user data
	// User data is after the 2 added byte.
	for ( i=2; i<len+2; i++)
		rxb[i-2]=ptr[i];
	
	return len;
}



int32_t ADXL362_Write( uint8_t reg, uint8_t*txb, int32_t len)
{
	uint8_t ptr_tx[4];
	uint8_t ptr[4];
	uint8_t i;

	if ( len > 2)
	return 1;
	
	// buffer used to transmit CMD and Register
	ptr_tx[0] = ADXL362_WR_CMD;
	ptr_tx[1] = reg;
	
	// copy the user data to tx buffer
	// User data is after the 2 added byte.
	for ( i=0; i<len; i++)
	ptr_tx[i+2]=txb[i];
	
	//
	spi_select_slave(&ADXL362_spi_master_instance, &ADXL362_slave, true);
	
	ADSL362_spi_ret = spi_transceive_buffer_job(&ADXL362_spi_master_instance, ptr_tx, ptr, len+2);
	// ADSL362_spi_ret = spi_write_buffer_job(&ADXL362_spi_master_instance, ptr_tx, len+2);
	
	//
	if ( ADSL362_spi_ret != 0) {
		spi_select_slave(&ADXL362_spi_master_instance, &ADXL362_slave, false);
		return ADSL362_spi_ret;
	}
	
	while (!ADXL362_trans_complete_spi_master) {
		/////* Wait for write and read complete */
	}
	ADXL362_trans_complete_spi_master = false;
	
	spi_select_slave(&ADXL362_spi_master_instance, &ADXL362_slave, false);
	
	return len;
}

/**
 * \brief Low Level Call to Reads multiple bytes from the device's FIFO buffer.
 *
 * \param rxb   Stores the read bytes.
 * \param len   buffer lenght
 * \return      Correct operation.
 * \retval  0 Operation OK
 * \retval  1 Error 
 */
int32_t ADXL362_ReadFifo( uint8_t*rxb, int32_t len)
{
	uint8_t ptr[1024+1];
	uint8_t ptr_tx[1024+1];
	uint32_t i;

    if ( len > 512)
        return 1;
        
	// buffer used to transmit CMD and Register	
	ptr_tx[0] = ADXL362_FIFO_CMD;
	
    //
	spi_select_slave(&ADXL362_spi_master_instance, &ADXL362_slave, true);
	
	ADSL362_spi_ret = spi_transceive_buffer_job(&ADXL362_spi_master_instance, ptr_tx, ptr, (len*2)+1);
	
	if ( ADSL362_spi_ret != 0) {
		spi_select_slave(&ADXL362_spi_master_instance, &ADXL362_slave, false);	
		return ADSL362_spi_ret;
	}
	
	while (!ADXL362_trans_complete_spi_master) {
		/////* Wait for write and read complete */
	}
	ADXL362_trans_complete_spi_master = false;

	spi_select_slave(&ADXL362_spi_master_instance, &ADXL362_slave, false);	
	
	// copy the returned data to user data
	// User data is after the 2 added byte.
	for ( i=0; i<(len*2); i++)
		rxb[i]=ptr[i+1];
	
	return 0;
}










/* */
#if 0
int32_t ADXL362_Read( uint8_t reg, uint8_t*rxb, int32_t len)
{
	//uint8_t buff[2];
	uint8_t*ptr;
	uint8_t*ptr_tx;
	uint8_t i;

	// buffer used to transmit CMD and Register	
	ptr_tx=malloc(len+2);
	if ( ptr_tx==(uint8_t*)NULL)
		return 1;

	ptr_tx[0] = ADXL362_RD_CMD;
	ptr_tx[1] = reg;
	
	// buffer used to receive the data from the ADXL362
	// the received buffer must be 2 byte more long. User len plus CMD and register byte.
	ptr=malloc(len+2);
	if ( ptr==(uint8_t*)NULL) {
		free( ptr_tx);
		return 1;
	}
		
	spi_select_slave(&ADXL362_spi_master_instance, &ADXL362_slave, true);
	
	ADSL362_spi_ret = spi_transceive_buffer_job(&ADXL362_spi_master_instance, ptr_tx, ptr, len+2);
	
	if ( ADSL362_spi_ret != 0) {
		spi_select_slave(&ADXL362_spi_master_instance, &ADXL362_slave, false);	
		free( ptr);
		free( ptr_tx);
		return ADSL362_spi_ret;
	}
	
	while (!ADXL362_trans_complete_spi_master) {
		/////* Wait for write and read complete */
	}
	ADXL362_trans_complete_spi_master = false;

	spi_select_slave(&ADXL362_spi_master_instance, &ADXL362_slave, false);	
	
	// copy the returned data to user data
	// User data is after the 2 added byte.
	for ( i=2; i<len+2; i++)
		rxb[i-2]=ptr[i];

	free(ptr);
	free(ptr_tx);
	
	return len;
}




int32_t ADXL362_Write( uint8_t reg, uint8_t*txb, int32_t len)
{
	//uint8_t buff[2];
	uint8_t*ptr;
	uint8_t*ptr_tx;
	uint8_t i;

	// buffer used to transmit CMD and Register
	ptr_tx=malloc(len+2);
	if ( ptr_tx==(uint8_t*)NULL)
	return 1;

	ptr_tx[0] = ADXL362_WR_CMD;
	ptr_tx[1] = reg;
	// copy the user data to tx buffer
	// User data is after the 2 added byte.
	for ( i=0; i<len; i++)
		ptr_tx[i+2]=txb[i];
	
	// dummy buffer used to receive the data from the ADXL362
	// the received buffer must be 2 byte more long. User len plus CMD and register byte.
	ptr=malloc(len+2);
	if ( ptr==(uint8_t*)NULL)
	return 1;

	spi_select_slave(&ADXL362_spi_master_instance, &ADXL362_slave, true);
	
	ADSL362_spi_ret = spi_transceive_buffer_job(&ADXL362_spi_master_instance, ptr_tx, ptr, len+2);
	//ADSL362_spi_ret = spi_write_buffer_job(&ADXL362_spi_master_instance, txb, len);
	
	//
	if ( ADSL362_spi_ret != 0) {
		spi_select_slave(&ADXL362_spi_master_instance, &ADXL362_slave, false);
		free( ptr);
		return ADSL362_spi_ret;
	}
	
	while (!ADXL362_trans_complete_spi_master) {
		/////* Wait for write and read complete */
	}
	ADXL362_trans_complete_spi_master = false;
	
	spi_select_slave(&ADXL362_spi_master_instance, &ADXL362_slave, false);
	free( ptr);
	
	return len;
}
#endif

