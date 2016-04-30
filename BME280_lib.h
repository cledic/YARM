/*
*/
#define BME280_ADDR				(0x76)	
#define BME280_CHIPID			(0x60)
#define BME280_REG_ID			(0xD0)
//
#define BME280_REG_HUM_LSB 		(0xFE)
#define BME280_REG_HUM_MSB 		(0xFD)
#define BME280_REG_TEMP_XLSB 	(0xFC)
#define BME280_REG_TEMP_LSB 	(0xFB)
#define BME280_REG_TEMP_MSB 	(0xFA)
#define BME280_REG_PRESS_XSLB 	(0xF9)
#define BME280_REG_PRESS_LSB 	(0xF8)
#define BME280_REG_PRESS_MSB 	(0xF7)
//
#define BME280_REG_CONFIG 		(0xF5)
#define BME280_REG_CTRL_MEAS 	(0xF4)
#define BME280_REG_STATUS 		(0xF3)
#define BME280_REG_CTRL_HUM 	(0xF2)
#define BME280_REG_CALIB2		(0xE1)		//calib26..calib41 	0xE1…0xF0 individual
#define BME280_REG_RESET 		(0xE0)
#define BME280_REG_CALIB1		(0x88)		//calib00..calib25 	0x88…0xA1
//
#define BME280_HUM_SZ			(2)
#define BME280_TEMP_SZ			(3)
#define BME280_PRESS_SZ			(3)
#define BME280_CALIB2_SZ		(7)			// 7 byte
#define BME280_CALIB1_SZ		(25)		// 25 bytes

/* */
#define BME280_REG_CTRL_MEAS			(0xF4)
#define BME280_PRESS_OVRSMPL_SKIPPED	(0x00)
#define BME280_PRESS_OVRSMPL_x1			(0x01<<2)
#define BME280_PRESS_OVRSMPL_x2			(0x02<<2)
#define BME280_PRESS_OVRSMPL_x4			(0x03<<2)
#define BME280_PRESS_OVRSMPL_x8			(0x04<<2)
#define BME280_PRESS_OVRSMPL_x16		(0x05<<2)
#define BME280_PRESS_OVRSMPL_MASK		(0x07<<2)
//
#define BME280_TEMP_OVRSMPL_SKIPPED		(0x00)
#define BME280_TEMP_OVRSMPL_x1			(0x01<<5)
#define BME280_TEMP_OVRSMPL_x2			(0x02<<5)
#define BME280_TEMP_OVRSMPL_x4			(0x03<<5)
#define BME280_TEMP_OVRSMPL_x8			(0x04<<5)
#define BME280_TEMP_OVRSMPL_x16			(0x05<<5)
#define BME280_TEMP_OVRSMPL_MASK		(0x07<<5)
//
#define BME280_MODE_SLEEP				(0x00)
#define BME280_MODE_FORCED				(0x01)
#define BME280_MODE_NORMAL				(0x03)
#define BME280_MODE_MASK				(0x03)

/* */
#define BME280_REG_CONFIG				(0xF5)
#define BME280_TIME_SNDBY_0_5			(0x00<<5)
#define BME280_TIME_SNDBY_62_5			(0x01<<5)
#define BME280_TIME_SNDBY_125			(0x02<<5)
#define BME280_TIME_SNDBY_250			(0x03<<5)
#define BME280_TIME_SNDBY_500			(0x04<<5)
#define BME280_TIME_SNDBY_1000			(0x05<<5)
#define BME280_TIME_SNDBY_10			(0x06<<5)
#define BME280_TIME_SNDBY_20			(0x07<<5)
#define BME280_TIME_MASK				(0x07<<5)
//
#define BME280_FLTRCOEFF_OFF			(0x00<<2)
#define BME280_FLTRCOEFF_2				(0x01<<2)
#define BME280_FLTRCOEFF_4				(0x02<<2)
#define BME280_FLTRCOEFF_8				(0x03<<2)
#define BME280_FLTRCOEFF_16				(0x04<<2)
#define BME280_FLTRCOEFF_MASK			(0x07<<2)

/* */
#define BME280_REG_RESET				(0xE0)
#define BME280_RESET					(0xB6)

/* */
#define BME280_REG_CTRL_HUM				(0xF2)
#define BME280_HUM_OVRSMPL_SKIPPED		(0x00)
#define BME280_HUM_OVRSMPL_x1			(0x01)
#define BME280_HUM_OVRSMPL_x2			(0x02)
#define BME280_HUM_OVRSMPL_x4			(0x03)
#define BME280_HUM_OVRSMPL_x8			(0x04)
#define BME280_HUM_OVRSMPL_x16			(0x05)
#define BME280_HUM_OVRSMPL_MASK			(0x07)

/* */
#define BME280_REG_STATUS				(0xF3)
#define BME280_MEASURING_MASK			(0x04)
#define BME280_UPDATE_MASK				(0x01)

/* */
#define BME280_PROFILE_WEATHER			(0x01)
#define BME280_PROFILE_HUMIDITY			(0x02)
#define BME280_PROFILE_INDOOR			(0x03)
#define BME280_PROFILE_GAMING			(0x04)

/* */
#define BME280_GET_HUMIDITY				(1)
#define BME280_GET_PRESSURE				(2)
#define BME280_GET_TEMPERATURE			(3)

typedef struct _CALIBRATION_VALUES {
	uint16_t 	dig_T1;
	int16_t		dig_T2;
	int16_t		dig_T3;
	uint16_t	dig_P1;
	int16_t		dig_P2;
	int16_t		dig_P3;
	int16_t		dig_P4;
	int16_t		dig_P5;
	int16_t		dig_P6;
	int16_t		dig_P7;
	int16_t		dig_P8;
	int16_t		dig_P9;
	uint8_t		dig_H1;
	int16_t		dig_H2;
	uint8_t		dig_H3;
	int16_t		dig_H4;
	int16_t		dig_H5;
	int8_t		dig_H6;
} CALIBRATION_VALUES;

/**
 * \brief This function read the temperature values and return it as a float
 *        Apply the compensation values.
 *
 * \param[in] (float*)t		pointer to the return value
 * \return					Return 0 for success or other values for errors.
 */
int32_t BME280_Get_Temperature( float*t);

/**
 * \brief This function read the pressure values and return it as a float
 *        Apply the compensation values.
 *
 * \param[in] (float*)t		pointer to the return value
 * \return					Return 0 for success or other values for errors.
 */
int32_t BME280_Get_Pressure( float*p);

/**
 * \brief This function read the humidity values and return it as a float
 *        Apply the compensation values.
 *
 * \param[in] (float*)t		pointer to the return value
 * \return					Return 0 for success or other values for errors.
 */
int32_t BME280_Get_Humidity( float*h);

/**
 * \brief Initialize low lever I2C interface and read the BME280 chip ID
 *
 * \param[in] none
 * \return Correct identification.
 * \retval 0 Identification OK
 * \retval return the error code 
 */
int32_t BME280_Init(void);


/**
 * \brief	Set the Temperature Oversampling value.
 *			Possible values are: 
 *			BME280_TEMP_OVRSMPL_x1	
 *          BME280_TEMP_OVRSMPL_x2	
 *          BME280_TEMP_OVRSMPL_x4	
 *          BME280_TEMP_OVRSMPL_x8	
 *          BME280_TEMP_OVRSMPL_x16	
 *
 *			Set the oversampling value to BME280_TEMP_OVRSMPL_SKIPPED
 *			will stop the acquisition.
 * \param[in]	int32_t to	oversampling value
 * \return 		return 0 for success, or other values for error
 */
int32_t BME280_SetTempOversampling( int32_t to);

/**
 * \brief	Set the Pressure Oversampling value.
 *			Possible values are: 
 *			BME280_PRESS_OVRSMPL_x1	
 *          BME280_PRESS_OVRSMPL_x2	
 *          BME280_PRESS_OVRSMPL_x4	
 *          BME280_PRESS_OVRSMPL_x8	
 *          BME280_PRESS_OVRSMPL_x16	
 *          
 *			Set the oversampling value to BME280_PRESS_OVRSMPL_SKIPPED
 *			will stop the acquisition. 
 * \param[in]	int32_t po	oversampling value
 * \return 		return 0 for success, or other values for error
 */
int32_t BME280_SetPressOversampling( int32_t po);

/**
 * \brief	Set the Humidity Oversampling value.
 *			Possible values are: 
 *			BME280_HUM_OVRSMPL_x1	
 *          BME280_HUM_OVRSMPL_x2	
 *          BME280_HUM_OVRSMPL_x4	
 *          BME280_HUM_OVRSMPL_x8	
 *          BME280_HUM_OVRSMPL_x16	
 *          
 *			Set the oversampling value to BME280_HUM_OVRSMPL_SKIPPED 
 *			will stop the acquisition. 
 * \param[in]	int32_t ho	oversampling value
 * \return 		return 0 for success, or other values for error
 */
int32_t BME280_SetHumOversampling( int32_t ho);

/**
  * \brief	Set the Inactive Duration value.
 *			Possible values are: 
 *			BME280_TIME_SNDBY_0_5	
 *          BME280_TIME_SNDBY_62_5	
 *          BME280_TIME_SNDBY_125	
 *          BME280_TIME_SNDBY_250	
 *          BME280_TIME_SNDBY_500	
 *          BME280_TIME_SNDBY_1000	
 *          BME280_TIME_SNDBY_10	
 *          BME280_TIME_SNDBY_20	
 * \param[in]	int32_t id	inactive duration value
 * \return 		return 0 for success, or other values for error
 */
int32_t BME280_SetInactiveDuration( int32_t id);

/**
  * \brief	Set the Filter Coefficient value.
 *			Possible values are: 
 *			BME280_FLTRCOEFF_OFF	
 *          BME280_FLTRCOEFF_2		
 *          BME280_FLTRCOEFF_4		
 *          BME280_FLTRCOEFF_8		
 *          BME280_FLTRCOEFF_16		
 *
 * \param[in]	int32_t fc	coefficient value
 * \return 		return 0 for success, or other values for error
 */
int32_t BME280_SetFilterCoeff( int32_t fc);

/**
  * \brief	Set the Sensor Mode value.
 *			Possible values are: 
 *			BME280_MODE_SLEEP	(no operation, all registers accessible, lowest power, selected after startup)
 *          BME280_MODE_FORCED	(perform one measurement, store results and return to sleep mode)
 *          BME280_MODE_NORMAL	(perpetual cycling of measurements and inactive periods)
 *
 * \param[in]	int32_t sm	mode value
 * \return 		return 0 for success, or other values for error
 */
int32_t BME280_SetSensorMode( int32_t sm);

/**
  * \brief	Get the Sensor Mode value.
 *			Returned values are: 
 *			BME280_MODE_SLEEP	0x00  (no operation, all registers accessible, lowest power, selected after startup)
 *          BME280_MODE_FORCED	0x01, 0x02 (perform one measurement, store results and return to sleep mode)
 *          BME280_MODE_NORMAL	0x03  (perpetual cycling of measurements and inactive periods)
 *
 * \param[in]	int32_t*sm	returned mode value
 * \return 		return 0 for success, or other values for error
 */
int32_t BME280_GetSensorMode( int32_t*sm);

/**
  * \brief	Set a Profile for the device.
 *			Possible values are: 
 *			BME280_PROFILE_WEATHER		(0.16 uA)
 *			BME280_PROFILE_HUMIDITY		(2.9  uA)
 *			BME280_PROFILE_INDOOR		(633  uA)
 *			BME280_PROFILE_GAMING		(581  uA)
 *
 * \param[in]	int32_t p	profile 
 * \return 		return 0 for success, or other values for error
 */
int32_t BME280_SetProfile( int32_t p);

/**
 * \brief This function read the status register
 *
 * \param[in] (int32_t*)s	pointer to the status variable
 * \return					Return 0 for success or other values for errors.
 */
 int32_t BME280_GetStatus( int32_t*s);


/**
 * \brief This function read all the parameter and return their raw values
 *        No compensation applied
 *
 * \param[in] (uint32_t*)press		pointer to the return value
 * \param[in] (uint32_t*)temp		pointer to the return value
 * \param[in] (uint32_t*)hum		pointer to the return value
 * \return					Return 0 for success or other values for errors.
 */
int32_t BME280_GetRawValues( uint32_t*press, uint32_t*temp, uint32_t*hum);

/**
 * \brief This function read all the parameter and return their values
  *
 * \param[in] (float*)press		pointer to the return value
 * \param[in] (float*)temp		pointer to the return value
 * \param[in] (float*)hum		pointer to the return value
 * \return					Return 0 for success or other values for errors.
 */
int32_t BME280_Get_AllValues( float*p, float*t, float*h);

int32_t BME280_Get_SingleValue( uint32_t m, float*value);

/*
 * Suggested settings for weather monitoring
 * Sensor mode 				forced mode, 1 sample / minute
 * Oversampling settings 	pressure x1, temperature x1, humidity x1
 * IIR filter settings 		filter off
 * Performance for suggested settings
 * Current consumption 0.16 uA
 * RMS Noise 3.3 Pa / 30 cm, 0.07 %RH
 * Data output rate 1/60 Hz
*/

/*
 * Suggested settings for humidity monitoring
 * Sensor mode 				forced mode, 1 sample / second
 * Oversampling settings 	pressure x0, temperature x1, humidity x1
 * IIR filter settings 		filter off
 * Performance for suggested settings
 * Current consumption 2.9 uA
 * RMS Noise 0.07 %RH
 * Data output rate 1 Hz
*/

/*
 * Suggested settings for indoor navigation
 * Sensor mode 			normal mode, tstandby = 0.5 ms
 * Oversampling settings 	pressure x16, temperature x2, humidity x1
 * IIR filter settings 	filter coefficient 16
 * Performance for suggested settings
 * Current consumption 633 uA
 * RMS Noise 0.2 Pa / 1.7 cm
 * Data output rate 25Hz
 * Filter bandwidth 0.53 Hz
 * Response time (75%) 0.9 s
*/

/*
 * Suggested settings for gaming
 * Sensor mode 			normal mode, tstandby = 0.5 ms
 * Oversampling settings	pressure x4, temperature x1, humidity x0
 * IIR filter settings 	filter coefficient 16
 * Performance for suggested settings
 * Current consumption 581 uA
 * RMS Noise 0.3 Pa / 2.5 cm
 * Data output rate 83 Hz
 * Filter bandwidth 1.75 Hz
 * Response time (75%) 0.3 s
*/
