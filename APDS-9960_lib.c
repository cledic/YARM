/*
*/
#include <asf.h>
#include "APDS-9960_lib.h"


/**
 * \brief Driver version
 */
#define APDS_9660_DRIVER_VERSION 0x00000001u
#define CONF_I2C_MASTER_MODULE    SERCOM2

struct i2c_master_module i2c_master_instance;

/**
 * \brief Private functions
 */
int32_t APDS_9660_read_RegisterMultiValue( struct i2c_master_module*module, uint8_t reg, uint8_t *value, const uint16_t n);
int32_t APDS_9660_write_RegisterMultiValue( struct i2c_master_module*module, uint8_t*value, const uint16_t n);

uint32_t			APDS_9660_Init_Done=0;
struct				i2c_master_packet packet;

void APDS_9660_configure_i2c_master(void);

/* Members */
Gesture_Data_Type GD;
int32_t gesture_ud_delta_;
int32_t gesture_lr_delta_;
int32_t gesture_ud_count_;
int32_t gesture_lr_count_;
int32_t gesture_near_count_;
int32_t gesture_far_count_;
int32_t gesture_state_;
int32_t gesture_motion_;   

void APDS_9660_configure_i2c_master(void)
{
	/* Initialize config structure and software module. */
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);

	/* Change buffer timeout to something longer. */
	config_i2c_master.buffer_timeout = 10000;

	/* Initialize and enable device with config. */
	i2c_master_init(&i2c_master_instance, CONF_I2C_MASTER_MODULE, &config_i2c_master);

	i2c_master_enable(&i2c_master_instance);
}

 uint32_t APDS_9660_Init( void)
 {
    uint8_t id;
    uint8_t val[2];
    
	APDS_9660_configure_i2c_master();
	
    // id=I2CreadByte(APDS9960_I2C_ADDR, APDS9960_ID);
    APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_ID, &id, 1);
    
    if( (!(id == APDS9960_ID_1 || id == APDS9960_ID_2))||id==ERROR) return 1;
    
    if(APDS_9660_SetMode(ALL, OFF)) return 1;

    val[0]=APDS9960_ATIME;
    val[1]=DEFAULT_ATIME;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;

    val[0]=APDS9960_WTIME;
    val[1]=DEFAULT_WTIME;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;

    val[0]=APDS9960_PPULSE;
    val[1]=DEFAULT_PROX_PPULSE;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;

    val[0]=APDS9960_POFFSET_UR;
    val[1]=DEFAULT_POFFSET_UR;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;

    val[0]=APDS9960_POFFSET_DL;
    val[1]=DEFAULT_POFFSET_DL;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;

    val[0]=APDS9960_CONFIG1;
    val[1]=DEFAULT_CONFIG1;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;

    if( APDS_9660_SetLEDDrive( DEFAULT_LDRIVE) ) return 1;
    if( APDS_9660_SetProximityGain( DEFAULT_PGAIN) ) return 1;
    if( APDS_9660_SetAmbientLightGain( DEFAULT_AGAIN) ) return 1;
    if( APDS_9660_SetProxIntLowThresh( DEFAULT_PILT) ) return 1;
    if( APDS_9660_SetProxIntHighThresh( DEFAULT_PIHT) ) return 1;
    if( APDS_9660_SetLightIntLowThreshold( DEFAULT_AILT) ) return 1;

    val[0]=APDS9960_CONFIG2;
    val[1]=DEFAULT_CONFIG2;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;    

    val[0]=APDS9960_CONFIG3;
    val[1]=DEFAULT_CONFIG3;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;    
    
    if( APDS_9660_SetGestureEnterThresh(DEFAULT_GPENTH) ) return 1;
    if( APDS_9660_SetGestureExitThresh(DEFAULT_GEXTH) ) return 1;
    
    val[0]=APDS9960_GCONF1;
    val[1]=DEFAULT_GCONF1;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;     

    if( APDS_9660_SetGestureGain( DEFAULT_GGAIN) ) return 1;
    if( APDS_9660_SetGestureLEDDrive( DEFAULT_GLDRIVE) ) return 1;
    if( APDS_9660_SetGestureWaitTime( DEFAULT_GWTIME) ) return 1;
    
    val[0]=APDS9960_GOFFSET_U;
    val[1]=DEFAULT_GOFFSET;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;         

    val[0]=APDS9960_GOFFSET_D;
    val[1]=DEFAULT_GOFFSET;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1; 
    
    val[0]=APDS9960_GOFFSET_L;
    val[1]=DEFAULT_GOFFSET;    
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1; 
    
    val[0]=APDS9960_GOFFSET_R;
    val[1]=DEFAULT_GOFFSET;        
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;   
    

    val[0]=APDS9960_GPULSE;
    val[1]=DEFAULT_GPULSE;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;  

    val[0]=APDS9960_GCONF3;
    val[1]=DEFAULT_GCONF3;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;  

    if( APDS_9660_SetGestureIntEnable( DEFAULT_GIEN) ) return 1;
    
	APDS_9660_Init_Done=1;
	
    return 0;

}

/**
 * @brief Enables or disables a feature in the APDS-9960
 *
 * @param[in] mode which feature to enable
 * @param[in] enable ON (1) or OFF (0)
 * @return 0 if operation success.  1 otherwise.
 */
uint32_t APDS_9660_SetMode(uint8_t mode, uint8_t enable)
{
    uint8_t reg_val;
    uint8_t val[2];

    /* Read current ENABLE register */
    if ( APDS_9660_GetMode( &reg_val)) return 1;
    
    /* Change bit(s) in ENABLE register */
    enable = enable & 0x01;
    if( mode >= 0 && mode <= 6 ) {
        if (enable) {
            reg_val |= (1 << mode);
        } else {
            reg_val &= ~(1 << mode);
        }
    } else if( mode == ALL ) {
        if (enable) {
            reg_val = 0x7F;
        } else {
            reg_val = 0x00;
        }
    }
    
    val[0]=APDS9960_ENABLE;
    val[1]=reg_val;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;  
        
    return 0;
}

uint32_t APDS_9660_GetMode( uint8_t*mode)
{
	
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_ENABLE, mode, 1)) return 1;

    return 0;
}



uint32_t APDS_9660_EnableLightSensor(uint8_t interrupts)
{
    
    /* Set default gain, interrupts, enable power, and enable sensor */
    if( APDS_9660_SetAmbientLightGain(DEFAULT_AGAIN) ) return 1;
    
    if( interrupts ) {
        if( APDS_9660_SetAmbientLightIntEnable(1) ) 
			return 1;
    } else {
        if( APDS_9660_SetAmbientLightIntEnable(0) ) 
			return 1;
    }
    if( APDS_9660_EnablePower() ) return 1;
    
    if( APDS_9660_SetMode(AMBIENT_LIGHT, 1)) return 1;
    
    return 0;

}

/**
 * @brief Ends the light sensor on the APDS-9960
 *
 * @return 0 if sensor disabled correctly. 1 on error.
 */
uint32_t APDS_9660_DisableLightSensor( void)
{
    if( APDS_9660_SetAmbientLightIntEnable(0) ) {
        return 1;
    }
    if( APDS_9660_SetMode(AMBIENT_LIGHT, 0) ) {
        return 1;
    }
    
    return 0;
}

/**
 * @brief Starts the proximity sensor on the APDS-9960
 *
 * @param[in] interrupts true to enable hardware external interrupt on proximity
 * @return 0 if sensor enabled correctly. 1 on error.
 */
uint32_t APDS_9660_EnableProximitySensor(uint8_t interrupts)
{
    /* Set default gain, LED, interrupts, enable power, and enable sensor */
    if( APDS_9660_SetProximityGain(DEFAULT_PGAIN) ) return 1;
    if( APDS_9660_SetLEDDrive(DEFAULT_LDRIVE) ) return 1;

    if( interrupts ) {
        if( APDS_9660_SetProximityIntEnable(1) ) {
            return 1;
        }
    } else {
        if( APDS_9660_SetProximityIntEnable(0) ) {
            return 1;
        }
    }
	
    if( APDS_9660_EnablePower() ) return 1;
    if( APDS_9660_SetMode(PROXIMITY, 1) ) return 1;
    
    return 0;
}

/**
 * @brief Ends the proximity sensor on the APDS-9960
 *
 * @return 0 if sensor disabled correctly. 1 on error.
 */
uint32_t APDS_9660_DisableProximitySensor( void)
{
    if( APDS_9660_SetProximityIntEnable(0) ) return 1;
    if( APDS_9660_SetMode(PROXIMITY, 0) ) return 1;

    return 0;
}


/**
 * @brief Starts the gesture recognition engine on the APDS-9960
 *
 * @param[in] interrupts true to enable hardware external interrupt on gesture
 * @return 0 if engine enabled correctly. 1 on error.
 */
uint32_t APDS_9660_EnableGestureSensor(uint8_t interrupts)
{
    uint8_t val[2];
	
    /* Enable gesture mode
       Set ENABLE to 0 (power off)
       Set WTIME to 0xFF
       Set AUX to LED_BOOST_300
       Enable PON, WEN, PEN, GEN in ENABLE 
    */

    APDS_9660_ResetGestureParameters();

    val[0]=APDS9960_WTIME;
    val[1]=0xFF;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;	

    val[0]=APDS9960_PPULSE;
    val[1]=DEFAULT_GESTURE_PPULSE;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;	

    if( APDS_9660_SetLEDBoost(LED_BOOST_300) ) return 1;

    if( interrupts ) {
        if( APDS_9660_SetGestureIntEnable(1) ) {
            return 1;
        }
    } else {
        if( APDS_9660_SetGestureIntEnable(0) ) {
            return 1;
        }
    }

    if( APDS_9660_SetGestureMode(1) ) return 1;
    if( APDS_9660_EnablePower() ) return 1;
    if( APDS_9660_SetMode(WAIT, 1) ) return 1;
    if( APDS_9660_SetMode(PROXIMITY, 1) ) return 1;
    if( APDS_9660_SetMode(GESTURE, 1) ) return 1;
    
    return 0;
}

/**
 * @brief Ends the gesture recognition engine on the APDS-9960
 *
 * @return 0 if engine disabled correctly. 1 on error.
 */
uint32_t APDS_9660_DisableGestureSensor( void)
{
    APDS_9660_ResetGestureParameters();
    if( APDS_9660_SetGestureIntEnable(0) ) return 1;
    if( APDS_9660_SetGestureMode(0) ) return 1;
    if( APDS_9660_SetMode(GESTURE, 0) ) return 1;
    
    return 0;
}


/**
 * @brief Determines if there is a gesture available for reading
 *
 * @param[out] 1 if gesture available. 0 otherwise.
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_IsGestureAvailable( uint8_t*gest)
{
    uint8_t val;
    
    /* Read value from GSTATUS register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GSTATUS, &val, 1)) return 1;
    
    /* Shift and mask out GVALID bit */
    val &= APDS9960_GVALID;
    *gest=val;
	
	return 0;
}

int32_t APDS_9660_ReadGesture( uint8_t*dir)
{
    uint8_t fifo_level = 0;
    uint8_t fifo_data[128];
	char *fptr;
	fptr= fifo_data;
	  
    uint8_t gstatus;
	uint8_t mode, gesture;
    int motion;
    int i;
    
    /* Make sure that power and gesture is on and data is valid */
	if ( APDS_9660_GetMode(&mode)) return 1;
	if ( APDS_9660_IsGestureAvailable(&gesture)) return 1;
    if( !gesture || !( mode & 0x41) ) {
        *dir= DIR_NONE;
		return 0;
    }

    
    /* Keep looping as long as gesture data is valid */
    while(1) {
      
        /* Wait some time to collect next batch of FIFO data */
        delay_ms(FIFO_PAUSE_TIME);
        
        /* Get the contents of the STATUS register. Is data still valid? */
        if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GSTATUS, &gstatus, 1)) return 1;
        /* If we have valid data, read in FIFO */
        if( (gstatus & APDS9960_GVALID) == APDS9960_GVALID ) {
        
            /* Read the current FIFO level */
			if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GFLVL, &fifo_level, 1)) return 1;
            
            /* If there's stuff in the FIFO, read it into our data block */                 //NEED TO FIGURE OUT WHAT THIS IS DOING.
            if( fifo_level > 0) {
				if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GFIFO_U, &fifo_data[0], (fifo_level * 4))) return 1;

                /* If at least 1 set of data, sort the data into U/D/L/R */
                if((fifo_level * 4)  >= 4 ) {
                    for( i = 0; i < (fifo_level * 4); i += 4 ) {
                        GD.u_data[GD.sindex] = fifo_data[i + 0];
                        GD.d_data[GD.sindex] = fifo_data[i + 1];
                        GD.l_data[GD.sindex] = fifo_data[i + 2];
                        GD.r_data[GD.sindex] = fifo_data[i + 3];
                        GD.sindex++;
                        GD.total_gestures++;
                    }
                    

                    /* Filter and process gesture data. Decode near/far state */
                    APDS_9660_ProcessGestureData();
                    APDS_9660_DecodeGesture();
                    
                    /* Reset data */
                    GD.sindex = 0;
                    GD.total_gestures = 0;
                }
            }
        } else {
    
            /* Determine best guessed gesture and clean up */
            delay_ms(FIFO_PAUSE_TIME);
            APDS_9660_DecodeGesture();
            motion = gesture_motion_;
            APDS_9660_ResetGestureParameters();
			*dir=motion;
            
			return 0;
        }
    }	// while(1)...
   // 
   return 0;
}
/**
 * Turn the APDS-9960 on
 *
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_EnablePower( void)
{
    if( APDS_9660_SetMode(POWER, 1) ) return 1;
    
    return 0;
}

/**
 * Turn the APDS-9960 off
 *
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_DisablePower( void)
{
    if( APDS_9660_SetMode(POWER, 0) ) return 1;
    
    return 0;
}

/*******************************************************************************
 * Ambient light and color sensor controls
 ******************************************************************************/

/**
 * @brief Reads the ambient (clear) light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_ReadAmbientLight(uint16_t*val)
{
    uint8_t val_byte[2];
    
    /* Read value from clear channel, low byte register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_CDATAL, &val_byte[0], 1)) return 1;
    
    /* Read value from clear channel, high byte register */
   if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_CDATAH, &val_byte[1], 1)) return 1;

   *val = (uint16_t)(val_byte[0] + (val_byte[1] << 8));
    
    return 0;
}

/**
 * @brief Reads the red light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_ReadRedLight(uint16_t*val)
{
    uint8_t val_byte[2];
    // val = 0;
    
    /* Read value from clear channel, low byte register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_RDATAL, &val_byte[0], 1)) return 1;
    
    /* Read value from clear channel, high byte register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_RDATAH, &val_byte[1], 1)) return 1;

    *val = (uint16_t)(val_byte[0] + (val_byte[1] << 8));
    
    return 0;
}

/**
 * @brief Reads the green light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return 0 if operation successful. 1 otherwise.
 */

uint32_t APDS_9660_ReadGreenLight(uint16_t*val)
{
    uint8_t val_byte[2];
    //val = 0;
    
    /* Read value from clear channel, low byte register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GDATAL, &val_byte[0], 1)) return 1;
    
    /* Read value from clear channel, high byte register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GDATAH, &val_byte[1], 1)) return 1;

    *val = (uint16_t)(val_byte[0] + (val_byte[1] << 8));
    
    return 0;
}

/**
 * @brief Reads the red light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return 0 if operation successful. 1 otherwise.
*/

uint32_t APDS_9660_ReadBlueLight(uint16_t*val)
{
    uint8_t val_byte[2];
    //val = 0;
    
    /* Read value from clear channel, low byte register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_BDATAL, &val_byte[0], 1)) return 1;
    
    /* Read value from clear channel, high byte register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_BDATAH, &val_byte[1], 1)) return 1;

    *val = (uint16_t)(val_byte[0] + (val_byte[1] << 8));
    
    return 0;
}

/*******************************************************************************
 * Proximity sensor controls
 ******************************************************************************/

/**
 * @brief Reads the proximity level as an 8-bit value
 *
 * @param[out] val value of the proximity sensor.
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_ReadProximity(uint8_t*val)
{
    //val = 0;
    
    /* Read value from proximity data register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_PDATA, val, 1)) return 1;
    
    return 0;
}

/*******************************************************************************
 * High-level gesture controls
 ******************************************************************************/

/**
 * @brief Resets all the parameters in the gesture data member
 */
void APDS_9660_ResetGestureParameters( void)
{
    GD.sindex = 0;
    GD.total_gestures = 0;
    
    gesture_ud_delta_ = 0;
    gesture_lr_delta_ = 0;
    
    gesture_ud_count_ = 0;
    gesture_lr_count_ = 0;
    
    gesture_near_count_ = 0;
    gesture_far_count_ = 0;
    
    gesture_state_ = 0;
    gesture_motion_ = DIR_NONE;
}

uint32_t APDS_9660_ProcessGestureData( void)
{
    uint8_t u_first = 0;
    uint8_t d_first = 0;
    uint8_t l_first = 0;
    uint8_t r_first = 0;
    uint8_t u_last = 0;
    uint8_t d_last = 0;
    uint8_t l_last = 0;
    uint8_t r_last = 0;
    int ud_ratio_first;
    int lr_ratio_first;
    int ud_ratio_last;
    int lr_ratio_last;
    int ud_delta;
    int lr_delta;
    int i;

    /* If we have less than 4 total gestures, that's not enough */
    if( GD.total_gestures <= 4 ) {
        return false;
    }
    
    /* Check to make sure our data isn't out of bounds */
    if( (GD.total_gestures <= 32) && \
        (GD.total_gestures > 0) ) {
        
        /* Find the first value in U/D/L/R above the threshold */
        for( i = 0; i < GD.total_gestures; i++ ) {
            if( (GD.u_data[i] > GESTURE_THRESHOLD_OUT) &&
                (GD.d_data[i] > GESTURE_THRESHOLD_OUT) &&
                (GD.l_data[i] > GESTURE_THRESHOLD_OUT) &&
                (GD.r_data[i] > GESTURE_THRESHOLD_OUT) ) {
                
                u_first = GD.u_data[i];
                d_first = GD.d_data[i];
                l_first = GD.l_data[i];
                r_first = GD.r_data[i];
                break;
            }
        }
        
        /* If one of the _first values is 0, then there is no good data */
        if( (u_first == 0) || (d_first == 0) || \
            (l_first == 0) || (r_first == 0) ) {
            
            return false;
        }
        /* Find the last value in U/D/L/R above the threshold */
        for( i = GD.total_gestures - 1; i >= 0; i-- ) {
            if( (GD.u_data[i] > GESTURE_THRESHOLD_OUT) &&
                (GD.d_data[i] > GESTURE_THRESHOLD_OUT) &&
                (GD.l_data[i] > GESTURE_THRESHOLD_OUT) &&
                (GD.r_data[i] > GESTURE_THRESHOLD_OUT) ) {
                
                u_last = GD.u_data[i];
                d_last = GD.d_data[i];
                l_last = GD.l_data[i];
                r_last = GD.r_data[i];
                break;
            }
        }
    }
    
    /* Calculate the first vs. last ratio of up/down and left/right */
    ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first);
    lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first);
    ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last);
    lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last);
            
    /* Determine the difference between the first and last ratios */
    ud_delta = ud_ratio_last - ud_ratio_first;
    lr_delta = lr_ratio_last - lr_ratio_first;
    
    /* Accumulate the UD and LR delta values */
    gesture_ud_delta_ += ud_delta;
    gesture_lr_delta_ += lr_delta;
        
    /* Determine U/D gesture */
    if( gesture_ud_delta_ >= GESTURE_SENSITIVITY_1 ) {
        gesture_ud_count_ = 1;
    } else if( gesture_ud_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        gesture_ud_count_ = -1;
    } else {
        gesture_ud_count_ = 0;
    }
    
    /* Determine L/R gesture */
    if( gesture_lr_delta_ >= GESTURE_SENSITIVITY_1 ) {
        gesture_lr_count_ = 1;
    } else if( gesture_lr_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        gesture_lr_count_ = -1;
    } else {
        gesture_lr_count_ = 0;
    }
    
    /* Determine Near/Far gesture */
    if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 0) ) {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {
            
            if( (ud_delta == 0) && (lr_delta == 0) ) {
                gesture_near_count_++;
            } else if( (ud_delta != 0) || (lr_delta != 0) ) {
                gesture_far_count_++;
            }
            
            if( (gesture_near_count_ >= 10) && (gesture_far_count_ >= 2) ) {
                if( (ud_delta == 0) && (lr_delta == 0) ) {
                    gesture_state_ = NEAR_STATE;
                } else if( (ud_delta != 0) && (lr_delta != 0) ) {
                    gesture_state_ = FAR_STATE;
                }
                return true;
            }
        }
    } else {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {
                
            if( (ud_delta == 0) && (lr_delta == 0) ) {
                gesture_near_count_++;
            }
            
            if( gesture_near_count_ >= 5 ) {
                gesture_ud_count_ = 0;
                gesture_lr_count_ = 0;
                gesture_ud_delta_ = 0;
                gesture_lr_delta_ = 0;
            }
        }
    }
    
    
    return false;
}

/**
 * @brief Determines swipe direction or near/far state
 *
 * @return 1 if near/far event. 0 otherwise.
 */
uint32_t APDS_9660_DecodeGesture( void)
{
    /* Return if near or far event is detected */
    if( gesture_state_ == NEAR_STATE ) {
        gesture_motion_ = DIR_NEAR;
        return 1;
    } else if ( gesture_state_ == FAR_STATE ) {
        gesture_motion_ = DIR_FAR;
        return 1;
    }
    
    /* Determine swipe direction */
    if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 0) ) {
        gesture_motion_ = DIR_UP;
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 0) ) {
        gesture_motion_ = DIR_DOWN;
    } else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 1) ) {
        gesture_motion_ = DIR_RIGHT;
    } else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == -1) ) {
        gesture_motion_ = DIR_LEFT;
    } else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_UP;
        } else {
            gesture_motion_ = DIR_RIGHT;
        }
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == -1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_DOWN;
        } else {
            gesture_motion_ = DIR_LEFT;
        }
    } else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == -1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_UP;
        } else {
            gesture_motion_ = DIR_LEFT;
        }
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_DOWN;
        } else {
            gesture_motion_ = DIR_RIGHT;
        }
    } else {
        return 0;
    }
    
    return 1;
}

/*******************************************************************************
 * Getters and setters for register values
 ******************************************************************************/

/**
 * @brief Returns the lower threshold for proximity detection
 *
 * @param[out] the lower proximity threshold
 * @return 0 if operation successful. 1 otherwise.
 */
 
 uint8_t APDS_9660_GetProxIntLowThresh( uint8_t*thrs)
{
    //uint8_t val;
    
    /* Read value from PILT register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_PILT, thrs, 1)) return 1;
    
    return 0;
}
 
 /**
 * @brief Sets the lower threshold for proximity detection
 *
 * @param[in] threshold the lower proximity threshold
 * @return 0 if operation successful. 1 otherwise.
 */
 uint32_t APDS_9660_SetProxIntLowThresh(uint8_t threshold)
{
	uint8_t val[2];
	
    val[0] = APDS9960_PILT;
    val[1] = threshold;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;
    
    return 0;
}

/**
 * @brief Returns the high threshold for proximity detection
 *
 * @param[out] threshold the high proximity threshold 
 * @return 0 if operation successful. 1 otherwise.
 */
uint8_t APDS_9660_GetProxIntHighThresh( uint8_t*thrs)
{
    
    /* Read value from PIHT register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_PILT, thrs, 1)) return 1;   
    
    return 0;
}

/**
 * @brief Sets the high threshold for proximity detection
 *
 * @param[in] threshold the high proximity threshold
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetProxIntHighThresh(uint8_t threshold)
{
	uint8_t val[2];
	
    val[0] = APDS9960_PIHT;
    val[1] = threshold;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;
    
    return 0;
}

 /**
 * @brief Returns LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[out] led drive current 
 * @return 0 if operation successful. 1 otherwise.
 *
 */
uint8_t APDS_9660_GetLEDDrive( uint8_t*leddrv)
{
    uint8_t val;
    
    /* Read value from CONTROL register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_CONTROL, &val, 1)) return 1;
    
    /* Shift and mask out LED drive bits */
    *leddrv = (val >> 6) & 0x03;//0b00000011;
    
    return 0;
}
 
 /**
 * @brief Sets the LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value (0-3) for the LED drive strength
 * @return 0 if operation successful. 1 otherwise.
 */
 
uint32_t APDS_9660_SetLEDDrive(uint8_t drive)
{
    uint8_t val[2];
    
    /* Read value from CONTROL register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_CONTROL, &val[1], 1)) return 1;

    /* Set bits in register to given value */
    drive &= 0x03;
    drive = drive << 6;
    val[1] &= 0x3F;
    val[1] |= drive;
    
    /* Write register value back into CONTROL register */
    val[0] = APDS9960_CONTROL;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;	
    
    return 0;
}

/**
 * @brief Returns receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[out] receive gain 
 * @return 0 if operation successful. 1 otherwise.
 */
uint8_t APDS_9660_GetProximityGain(uint8_t*gain)
{
    uint8_t val;
    
    /* Read value from CONTROL register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_CONTROL, &val, 1)) return 1;
    
    /* Shift and mask out PDRIVE bits */
    *gain = (val >> 2) & 0x03;
    
    return 0;
}

/**
 * @brief Sets the receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetProximityGain(uint8_t drive)
{
    uint8_t val[2];
    
    /* Read value from CONTROL register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_CONTROL, &val[1], 1)) return 1;
    /* Set bits in register to given value */
    drive &=0x03;
    drive = drive << 2;
    val[1] &= 0xF3;
    val[1] |= drive;
    
    /* Write register value back into CONTROL register */
    val[0] = APDS9960_CONTROL;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;		

    return 0;
}

/**
 * @brief Returns receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @param[out] drive the value (0-3) for the gain
 * @return 0 if operation successful. 1 otherwise.
 */
uint8_t APDS_9660_GetAmbientLightGain(uint8_t*gain)
{
    uint8_t val;
    
    /* Read value from CONTROL register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_CONTROL, &val, 1)) return 1;
    
    /* Shift and mask out ADRIVE bits */
    *gain = val & 0x03;
    
    return 0;
}

/**
 * @brief Sets the receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetAmbientLightGain(uint8_t drive){

    uint8_t val[2];
    
    /* Read value from CONTROL register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_CONTROL, &val[1], 1)) return 1;

    /* Set bits in register to given value */
    drive &=0x03;
    drive = drive << 2;
    val[1] &= 0xF3;
    val[1] |= drive;
    
    /* Write register value back into CONTROL register */
    val[0]=APDS9960_CONTROL;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;	
  
	return 0;
}

/**
 * @brief Get the current LED boost value
 * 
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @param[out] the boost value (0-3)
 * @return 0 if operation successful. 1 otherwise.
 */
uint8_t APDS_9660_GetLEDBoost( uint8_t*boost) 
{
    uint8_t val;
    
    /* Read value from CONFIG2 register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_CONFIG2, &val, 1)) return 1;
    
    /* Shift and mask out LED_BOOST bits */
    *boost = (val >> 4) & 0x03;
    
    return 0;
}

/**
 * @brief Sets the LED current boost value
 *
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @param[in] drive the value (0-3) for current boost (100-300%)
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetLEDBoost(uint8_t boost)
{
    uint8_t val[2];
    
    /* Read value from CONFIG2 register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_CONFIG2, &val[1], 1)) return 1;
    
    /* Set bits in register to given value */
    boost &= 0x03;
    boost = boost << 4;
    val[1] &= 0xCF;
    val[1] |= boost;
    
    /* Write register value back into CONFIG2 register */
    val[0]=APDS9960_CONFIG2;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;    
    
    return 0;
}    

/**
 * @brief Gets proximity gain compensation enable
 *
 * @param[out] 1 if compensation is enabled. 0 if not.
 * @return 0 if operation successful. 1 otherwise.
 */
uint8_t APDS_9660_GetProxGainCompEnable( uint8_t*gain)
{
    uint8_t val;
    
    /* Read value from CONFIG3 register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_CONFIG3, &val, 1)) return 1;
    
    /* Shift and mask out PCMP bits */
    *gain = (val >> 5) & 0x01;
    
    return 0;
}

/**
 * @brief Sets the proximity gain compensation enable
 *
 * @param[in] enable 1 to enable compensation. 0 to disable compensation.
 * @return 0 if operation successful. 1 otherwise.
 */
 uint32_t APDS_9660_SetProxGainCompEnable(uint8_t enable)
{
    uint8_t val[2];
    
    /* Read value from CONFIG3 register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_CONFIG3, &val[1], 1)) return 1;
    
    /* Set bits in register to given value */
    enable &= 0x01;
    enable = enable << 5;
    val[1] &= 0xDF;
    val[1] |= enable;
    
    /* Write register value back into CONFIG3 register */
    val[0]=APDS9960_CONFIG3;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;
    
    return 0;
}

/**
 * @brief Gets the current mask for enabled/disabled proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @param[out] Current proximity mask for photodiodes.
 * @return 0 if operation successful. 1 otherwise.
 */
uint8_t APDS_9660_GetProxPhotoMask( uint8_t*proxym)
{
    uint8_t val;
    
    /* Read value from CONFIG3 register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_CONFIG3, &val, 1)) return 1;
    
    /* Mask out photodiode enable mask bits */
    *proxym = val & 0x0F;
    
    return 0;
}

/**
 * @brief Sets the mask for enabling/disabling proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @param[in] mask 4-bit mask value
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetProxPhotoMask(uint8_t mask)
{
    uint8_t val[2];
    
    /* Read value from CONFIG3 register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_CONFIG3, &val[1], 1)) return 1;
    
    /* Set bits in register to given value */
    mask &= 0x0F;
    val[1] &= 0xF0;
    val[1] |= mask;
    
    /* Write register value back into CONFIG3 register */
    val[0]=APDS9960_CONFIG3;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;	
    
    return 0;
}

/**
 * @brief Gets the entry proximity threshold for gesture sensing
 *
 * @patam[out] Current entry proximity threshold.
 * @return 0 if operation successful. 1 otherwise.
 */
uint8_t APDS_9660_GetGestureEnterThresh( uint8_t*thrs)
{
    
    /* Read value from GPENTH register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GPENTH, thrs, 1)) return 1;
    
    return 0;
}

/**
 * @brief Sets the entry proximity threshold for gesture sensing
 *
 * @param[in] threshold proximity value needed to start gesture mode
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetGestureEnterThresh(uint8_t threshold)
{
	uint8_t val[2];
	
    val[0]=APDS9960_GPENTH;
    val[1]=threshold;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;
    
    return 0;
}

/**
 * @brief Gets the exit proximity threshold for gesture sensing
 *
 * @param[out] Current exit proximity threshold.
 * @return 0 if operation successful. 1 otherwise.
 */
uint8_t APDS_9660_GetGestureExitThresh( uint8_t*thrs)
{
    
    /* Read value from GEXTH register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GEXTH, thrs, 1)) return 1;
    
    return 0;
}

/**
 * @brief Sets the exit proximity threshold for gesture sensing
 *
 * @param[in] threshold proximity value needed to end gesture mode
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetGestureExitThresh(uint8_t threshold)
{
	uint8_t val[2];
	
    val[0]=APDS9960_GEXTH;
    val[1]=threshold;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;	
    
    return 0;
}

/**
 * @brief Gets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[out] the current photodiode gain.
 * @return 0 if operation successful. 1 otherwise. 
 */
uint8_t APDS_9660_GetGestureGain( uint8_t*gain)
{
    uint8_t val;
    
    /* Read value from GCONF2 register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GCONF2, &val, 1)) return 1;
    
    /* Shift and mask out GGAIN bits */
    *gain = (val >> 5) & 0x03;
    
    return 0;
}

/**
 * @brief Sets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] gain the value for the photodiode gain
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetGestureGain(uint8_t gain)
{
    uint8_t val[2];
    
    /* Read value from GCONF2 register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GCONF2, &val[1], 1)) return 1;
    
    /* Set bits in register to given value */
    gain &= 0x03;
    gain = gain << 5;
    val[1] &= 0x9F;
    val[1] |= gain;
    
    /* Write register value back into GCONF2 register */
    val[0]=APDS9960_GCONF2;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;	
    
    return 0;
}

/**
 * @brief Gets the drive current of the LED during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @parain[out] the LED drive current value. 0xFF on error.
 * @return 0 if operation successful. 1 otherwise. 
 */
uint8_t APDS_9660_GetGestureLEDDrive( uint8_t*leddrv)
{
    uint8_t val;
    
    /* Read value from GCONF2 register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GCONF2, &val, 1)) return 1;
    
    /* Shift and mask out GLDRIVE bits */
    *leddrv = (val >> 3) & 0x03;
    
    return 0;
}

/**
 * @brief Sets the LED drive current during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value for the LED drive current
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetGestureLEDDrive(uint8_t drive)
{
    uint8_t val[2];
    
    /* Read value from GCONF2 register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GCONF2, &val[1], 1)) return 1;
    
    /* Set bits in register to given value */
    drive &= 0x03;
    drive = drive << 3;
    val[1] &= 0xE7;
    val[1] |= drive;
    
    /* Write register value back into GCONF2 register */
    val[0]=APDS9960_GCONF2;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;
    
    return 0;
}

/**
 * @brief Gets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @param[out] the current wait time between gestures. 0xFF on error.
 * @return 0 if operation successful. 1 otherwise.
 */
uint8_t APDS_9660_GetGestureWaitTime(uint8_t*time)
{
    uint8_t val;
    
    /* Read value from GCONF2 register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GCONF2, &val, 1)) return 1;
    
    /* Mask out GWTIME bits */
    *time = val & 0x07;
    
    return 0;
}

/**
 * @brief Sets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @param[in] the value for the wait time
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetGestureWaitTime(uint8_t time)
{
    uint8_t val[2];
    
    /* Read value from GCONF2 register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GCONF2, &val[1], 1)) return 1;
    
    /* Set bits in register to given value */
    time &= 0x07;
    val[1] &= 0xF8;
    val[1] |= time;
    
    /* Write register value back into GCONF2 register */
    val[0]=APDS9960_GCONF2;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;	

    return 0;
}

/**
 * @brief Gets the low threshold for ambient light interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_GetLightIntLowThreshold(uint16_t*threshold)
{
    uint8_t val_byte[2];
    //threshold = 0;
    
    /* Read value from ambient light low threshold, low byte register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_AILTL, &val_byte[0], 1)) return 1;
    
    /* Read value from ambient light low threshold, high byte register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_AILTH, &val_byte[1], 1)) return 1;

    *threshold = (uint16_t)(val_byte[0] + (val_byte[1] << 8));
    
    return 0;
}

/**
 * @brief Sets the low threshold for ambient light interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetLightIntLowThreshold(uint16_t threshold)
{
    uint8_t val_low;
    uint8_t val_high;
	uint8_t val[2];
    
    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;
    
    /* Write low byte */
    val[0]=APDS9960_AILTL;
    val[1]=val_low;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;	
    
    /* Write high byte */
    val[0]=APDS9960_AILTH;
    val[1]=val_high;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;		
    
    return 0;
}

/**
 * @brief Gets the high threshold for ambient light interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_GetLightIntHighThreshold(uint16_t*threshold)
{
    uint8_t val_byte[2];
    //threshold = 0;
    
    /* Read value from ambient light high threshold, low byte register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_AIHTL, &val_byte[0], 1)) return 1;
    
    /* Read value from ambient light high threshold, high byte register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_AIHTH, &val_byte[1], 1)) return 1;	

    *threshold = (uint16_t)(val_byte[0] + (val_byte[1] << 8));
    
    return 0;
}

/**
 * @brief Sets the high threshold for ambient light interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetLightIntHighThreshold(uint16_t threshold)
{
    uint8_t val_low;
    uint8_t val_high;
	uint8_t val[2];
    
    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;
    
    /* Write low byte */
    val[0]=APDS9960_AIHTL;
    val[1]=val_low;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;	
    
    /* Write high byte */
    val[0]=APDS9960_AIHTH;
    val[1]=val_high;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;		
    
    return 0;
}

/**
 * @brief Gets the low threshold for proximity interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_GetProximityIntLowThreshold( uint8_t*threshold)
{
    
    /* Read value from proximity low threshold register */
	if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_PILT, threshold, 1)) return 1;
    
    return 0;
}

/**
 * @brief Sets the low threshold for proximity interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetProximityIntLowThreshold(uint8_t threshold)
{
    uint8_t val[2];
	
    /* Write threshold value to register */
    val[0] = APDS9960_PILT;
    val[1] = threshold;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;	
    
    return 0;
}
    
/**
 * @brief Gets the high threshold for proximity interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_GetProximityIntHighThreshold(uint8_t*threshold)
{
    
    /* Read value from proximity low threshold register */
    if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_PIHT, threshold, 1)) return 1;
    
    return 0;
}

/**
 * @brief Sets the high threshold for proximity interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetProximityIntHighThreshold(uint8_t threshold)
{
    uint8_t val[2];
    
    /* Write threshold value to register */
    val[0] = APDS9960_PIHT;
    val[1] = threshold;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;	    
    
    return 0;
}

/**
 * @brief Gets if ambient light interrupts are enabled or not
 *
 * @param[out] 1 if interrupts are enabled, 0 if not.
 * @return 0 if operation successful. 1 otherwise. 
 */
uint8_t APDS_9660_GetAmbientLightIntEnable( uint8_t*enable)
{
    uint8_t val;
    
    /* Read value from ENABLE register */
    if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_ENABLE, &val, 1)) return 1;
    
    /* Shift and mask out AIEN bit */
    *enable = (val >> 4) & 0x01;
    
    return 0;
}

/**
 * @brief Turns ambient light interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetAmbientLightIntEnable(uint8_t enable)
{
    uint8_t val_bytes[2];
    
    /* Read value from ENABLE register */
    if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_ENABLE, &val_bytes[1], 1)) return 1;
    
    /* Set bits in register to given value */
    enable &= 0x01;
    enable = enable << 4;
    val_bytes[1] &= 0xEF;
    val_bytes[1] |= enable;
    
    /* Write register value back into ENABLE register */
    val_bytes[0] = APDS9960_ENABLE;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val_bytes[0], 2)) return 1;	    
    
    return 0;
}

/**
 * @brief Gets if proximity interrupts are enabled or not
 *
 * @param[out] 1 if interrupts are enabled, 0 if not.
 * @return 0 if operation successful. 1 otherwise. 
 */
uint8_t APDS_9660_GetProximityIntEnable( uint8_t*enable)
{
    uint8_t val;
    
    /* Read value from ENABLE register */
    if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_ENABLE, &val, 1)) return 1;
    
    /* Shift and mask out PIEN bit */
    *enable = (val >> 5) & 0x01;
    
    return 0;
}

/**
 * @brief Turns proximity interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetProximityIntEnable(uint8_t enable)
{
    uint8_t val[2];
    
    /* Read value from ENABLE register */
    if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_ENABLE, &val[1], 1)) return 1;
    
    /* Set bits in register to given value */
    enable &= 0x01;
    enable = enable << 5;
    val[1] &= 0xDF;
    val[1] |= enable;
    
    /* Write register value back into ENABLE register */
    val[0] = APDS9960_ENABLE;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;	    
    
    return 0;
}

/**
 * @brief Gets if gesture interrupts are enabled or not
 *
 * @param[out] 1 if interrupts are enabled, 0 if not.
 * @return 0 if operation successful. 1 otherwise.
 */
uint8_t APDS_9660_GetGestureIntEnable( uint8_t*enable)
{
    uint8_t val;
    
    /* Read value from GCONF4 register */
    if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GCONF4, &val, 1)) return 1;
    
    /* Shift and mask out GIEN bit */
    *enable = (val >> 1) & 0x01;
    
    return 0;
}

/**
 * @brief Turns gesture-related interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetGestureIntEnable(uint8_t enable)
{
    uint8_t val[2];
    
    /* Read value from GCONF4 register */
    if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GCONF4, &val[1], 1)) return 1;
    
    /* Set bits in register to given value */
    enable &= 0x01;
    enable = enable << 1;
    val[1] &= 0xFD;
    val[1] |= enable;
    
    /* Write register value back into GCONF4 register */
    val[0] = APDS9960_GCONF4;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;	    
    
    return 0;
}

/**
 * @brief Clears the ambient light interrupt
 *
 * @return 0 if operation completed successfully. 1 otherwise.
 */
uint32_t APDS_9660_ClearAmbientLightInt( void)
{
    uint8_t throwaway;
    
    if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_AICLEAR, &throwaway, 1)) return 1;
    
    return 0;
}

/**
 * @brief Clears the proximity interrupt
 *
 * @return 0 if operation completed successfully. 1 otherwise.
 */
uint32_t APDS_9660_ClearProximityInt( void)
{
    uint8_t throwaway;
    
    if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_PICLEAR, &throwaway, 1)) return 1;
    
    return 0;
}

/**
 * @brief Tells if the gesture state machine is currently running
 *
 * @return 1 if gesture state machine is running, 0 if not. 0xFF on error.
 */
uint8_t APDS_9660_GetGestureMode( uint8_t*gesture)
{
    uint8_t val;
    
    /* Read value from GCONF4 register */
    if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GCONF4, &val, 1)) return 1;
    
    /* Mask out GMODE bit */
    *gesture =  val & 0x01;
    
    return 0;
}

/**
 * @brief Tells the state machine to either enter or exit gesture state machine
 *
 * @param[in] mode 1 to enter gesture state machine, 0 to exit.
 * @return 0 if operation successful. 1 otherwise.
 */
uint32_t APDS_9660_SetGestureMode(uint8_t mode)
{
    uint8_t val[2];
    
    /* Read value from GCONF4 register */
    if ( APDS_9660_read_RegisterMultiValue( &i2c_master_instance, APDS9960_GCONF4, &val[1], 1)) return 1;
    
    /* Set bits in register to given value */
    mode &= 0x01;
    val[1] &= 0xFE;
    val[1] |= mode;
    
    /* Write register value back into GCONF4 register */
    val[0] = APDS9960_GCONF4;
    if ( APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2)) return 1;	    
    
    return 0;
}

/*
 *	val[0] = 0xF4;
 *  val[1] = 0x2E;
 *
 * APDS_9660_write_RegisterMultiValue( &i2c_master_instance, &val[0], 2);
*/
int32_t APDS_9660_write_RegisterMultiValue( struct i2c_master_module*module, uint8_t*value, const uint16_t n)
{
	uint8_t r;
	
	packet.address     	= APDS9960_ADDR,
	packet.data_length 	= n,
	packet.data        	= value,
	packet.ten_bit_address = false,
	packet.high_speed      = false,
	packet.hs_master_code  = 0x0,

	r=i2c_master_write_packet_wait( module, &packet);

	return r;
}

/*
 * APDS_9660_read_RegisterMultiValue( &i2c_master_instance, 0x01, &val[0], 2);
*/
int32_t APDS_9660_read_RegisterMultiValue( struct i2c_master_module*module, uint8_t reg, uint8_t *value, const uint16_t n)
{
	uint8_t tmpbuffer[1];
	uint8_t r;
	
	tmpbuffer[0] = reg;
	
	packet.address     	= APDS9960_ADDR,
	packet.data_length 	= 1,
	packet.data        	= tmpbuffer,
	packet.ten_bit_address = false,
	packet.high_speed      = false,
	packet.hs_master_code  = 0x0,

	r=i2c_master_write_packet_wait_no_stop( module, &packet);
	if ( r != STATUS_OK) return r;

	packet.address     	= APDS9960_ADDR,
	packet.data_length 	= n,
	packet.data        	= value,
	packet.ten_bit_address = false,
	packet.high_speed      = false,
	packet.hs_master_code  = 0x0,
	
	r=i2c_master_read_packet_wait( module, &packet);
	
	return r;
	
}


