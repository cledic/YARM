/*
 * Simple library for Daisy24 LCD froma ACME Systems
 * http://www.acmesystems.it/DAISY-24
*/
#include <asf.h>

#include "Daisy24_lib.h"

/**
 * \brief Driver version
 */
#define Daisy24_DRIVER_VERSION 0x00000001u

#define CONF_I2C_MASTER_MODULE    SERCOM2

struct i2c_master_module i2c_master_instance;

//#define EXPANDER_I2C_ADDR       0x27    // PCF8574T
#define EXPANDER_I2C_ADDR     0x3F	// PCF8574AT
#define LCD_I2C_ADDR            0x3E //

typedef struct _MONITOR {
	char*p1;
	char*p2;
	uint8_t p1_idx;
	uint8_t p2_idx;
	uint8_t p1_len;
	uint8_t p2_len;
	uint8_t mode;
	uint8_t init;
} Monitor;

Monitor mn;

/**
 * \brief Private functions
 */
int32_t Daisy24_read_RegisterMultiValue( struct i2c_master_module*module, uint8_t i2c_addr, uint8_t reg, uint8_t *value, const uint16_t n);
int32_t Daisy24_write_RegisterMultiValue( struct i2c_master_module*module, uint8_t i2c_addr, uint8_t*value, const uint16_t n);

uint32_t			Daisy24_Init_Done=0;
int32_t				t_fine;
uint8_t				buffer[4];
uint8_t				reg_values[4];
struct i2c_master_packet packet;

void Daisy24_configure_i2c_master(void);

void Daisy24_configure_i2c_master(void)
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

/**
 * \brief Initialize low lever I2C interface and inizialize the LCD
 *
 * \param[in] none
 * \retval 0 OK
 * \retval return the error code 
 */
uint32_t Daisy24_LCD_Init(void)
{
    uint8_t buff[2];
    uint8_t r;
	
	if ( Daisy24_Init_Done==0)
        Daisy24_configure_i2c_master();	
    Daisy24_Init_Done=1;
	
    buff[0]=0x00;
    buff[1]=0x38;
    r=Daisy24_write_RegisterMultiValue( &i2c_master_instance, LCD_I2C_ADDR, &buff[0], 2);
	if ( r != STATUS_OK) return r;
	
    buff[1]=0x39;
    r=Daisy24_write_RegisterMultiValue( &i2c_master_instance, LCD_I2C_ADDR, &buff[0], 2);
	if ( r != STATUS_OK) return r;
	
    buff[1]=0x14;
    r=Daisy24_write_RegisterMultiValue( &i2c_master_instance, LCD_I2C_ADDR, &buff[0], 2);
	if ( r != STATUS_OK) return r;
	
    buff[1]=0x72;
    r=Daisy24_write_RegisterMultiValue( &i2c_master_instance, LCD_I2C_ADDR, &buff[0], 2);
	if ( r != STATUS_OK) return r;
	
    buff[1]=0x54;
    r=Daisy24_write_RegisterMultiValue( &i2c_master_instance, LCD_I2C_ADDR, &buff[0], 2);
	if ( r != STATUS_OK) return r;
	
    buff[1]=0x6F;
    r=Daisy24_write_RegisterMultiValue( &i2c_master_instance, LCD_I2C_ADDR, &buff[0], 2);
	if ( r != STATUS_OK) return r;
	
    buff[1]=0x0C;
    r=Daisy24_write_RegisterMultiValue( &i2c_master_instance, LCD_I2C_ADDR, &buff[0], 2);
    if ( r != STATUS_OK) return r;
	
	Daisy24_LCD_Clear();
	Daisy24_LCD_SetCursor( 0, 0);
	mn.init=0;
	
    return 0;
}

/**
 * \brief Initialize I2C Expander
 *
 * \param[in] none
 * \retval 0 OK
 * \retval return the error code 
 */
uint32_t Daisy24_Expander_Init(void)
{
	if ( Daisy24_Init_Done==0)
        Daisy24_configure_i2c_master();	
        
    Daisy24_Init_Done=1;
	
    return 0;
}

/**
 * \brief Clear LCD
 *
 * \param[in] none
 * \retval 0 OK 1 error
 */ 
uint32_t Daisy24_LCD_Clear(void)
{
    uint8_t buff[2];
    uint8_t r;
	
	if ( Daisy24_Init_Done==0)
        return 1;
	
    buff[0]=0x00;
    buff[1]=0x01;
    r=Daisy24_write_RegisterMultiValue( &i2c_master_instance, LCD_I2C_ADDR, &buff[0], 2);
    if ( r != STATUS_OK) return r;
	
	Daisy24_LCD_SetCursor( 0, 0);
	
    return 0;
}

/**
 * \brief LCD set cursor position
 *
 * \param[in] x and y position
 * \retval 0 OK 1 error
 */ 
uint32_t Daisy24_LCD_SetCursor( uint8_t x, uint8_t y)
{
    uint8_t buff[2];
    uint8_t r;
	
	if ( Daisy24_Init_Done==0)
        return 1;
	
    if ( y>1 || x>15)
        return 1;
    
    buff[0]=0x00;
    buff[1]=0x80 + (y*0x40) + x;
    r=Daisy24_write_RegisterMultiValue( &i2c_master_instance, LCD_I2C_ADDR, &buff[0], 2);
    if ( r != STATUS_OK) return r;
	
    return 0;
}

/**
 * \brief LCD write character
 *
 * \param[in] character to write
 * \retval 0 OK 1 error
 */ 
uint32_t Daisy24_LCD_WriteChar( char c)
{
    uint8_t buff[2];
    uint8_t r;
	
	if ( Daisy24_Init_Done==0)
        return 1;
	        
    buff[0]=0x40;
    buff[1]=c;
    r=Daisy24_write_RegisterMultiValue( &i2c_master_instance, LCD_I2C_ADDR, &buff[0], 2);
    if ( r != STATUS_OK) return r;
	
    return 0;
}

/**
 * \brief LCD write string
 *
 * \param[in] string to write
 * \retval 0 OK 1 error
 */ 
uint32_t Daisy24_LCD_WriteString( char*s)
{
    uint8_t i, l;
	
	if ( Daisy24_Init_Done==0)
        return 1;
	        
    if ( s==(char*)NULL)
        return 1;
        
    l=strlen(s);
    
    if( l > 16)
        l=16;
    
    for ( i=0; i<l; i++)
        Daisy24_LCD_WriteChar( s[i]);

	for( ; i<16;i++)
		Daisy24_LCD_WriteChar( ' ');
		
    return 0;
} 

/**
 * \brief LCD backLight ON
 *
 * \param[in] none
 * \retval 0 OK 1 error
 */ 
uint32_t Daisy24_LCD_BkLightOn(void)
{
    uint8_t buff[1];
    uint8_t r;
	
	if ( Daisy24_Init_Done==0)
        return 1;
	
    buff[0]=0x10;
    r=Daisy24_write_RegisterMultiValue( &i2c_master_instance, EXPANDER_I2C_ADDR, &buff[0], 1);
    if ( r != STATUS_OK) return r;
	
    return 0;
}

/**
 * \brief LCD backLight OFF
 *
 * \param[in] none
 * \retval 0 OK 1 error
 */ 
uint32_t Daisy24_LCD_BkLightOff(void)
{
    uint8_t buff[1];
    uint8_t r;
	
	if ( Daisy24_Init_Done==0)
        return 1;
	
    buff[0]=0x00;
    r=Daisy24_write_RegisterMultiValue( &i2c_master_instance, EXPANDER_I2C_ADDR, &buff[0], 1);
    if ( r != STATUS_OK) return r;
	
    return 0;
}

/**
 * \brief LCD Monitor, display two string with different effect
 * \param	s1, s2, m
 * \retval  0 OK, 1 Error
 * 
 * s1 is on he first line, s2 on the second
 * m is the display effect :
 * m == 1 s1 and s2 are both fixed on the LCD
 * m == 2 s1 is looping, s2 is fixed
 * m == 3 s1 is fixed, s2 is looping
 * m == 4 s1 and s2 both looping
*/
uint32_t Daisy24_LCD_MonitorInit( char*s1, char*s2, uint32_t m)
{
    
    if ( s1!=(char*)NULL && s2!=(char*)NULL) {
        mn.init=1;
        
        mn.p1=s1;
        mn.p2=s2;
        
        mn.p1_len=strlen(mn.p1);
        mn.p2_len=strlen(mn.p2);
        
        mn.mode=m;
        
        mn.p1_idx=0;
        mn.p2_idx=0;
        
        Daisy24_LCD_Clear();
        Daisy24_LCD_SetCursor( 0, 0);        
        Daisy24_LCD_WriteString(mn.p1);
        Daisy24_LCD_SetCursor( 0, 1);        
        Daisy24_LCD_WriteString(mn.p2);
		
		return 0;
    } else
        return 1;
}

/**
 * \brief LCD Monitor Update, called at time interval, update the string based on the effect
 *        configured.
 * \param[out]	f return the number of time the strings loop
 * \retval  0 OK, 1 Error
 * 
 */
uint32_t Daisy24_LCD_MonitorUpdate( uint32_t*f)
{
    uint8_t i, l, t;
	uint32_t c;
    
	c=*f;
	
    if ( mn.init==0)
        return 1;
    
	// mode 1 both strings fixed
    if ( mn.mode==1) {
        Daisy24_LCD_SetCursor( 0, 0);
        Daisy24_LCD_WriteString(mn.p1);
        Daisy24_LCD_SetCursor( 0, 1);
        Daisy24_LCD_WriteString(mn.p2);				
	}
    
	// mode 2 s1 looping, s2 fixed
    if ( mn.mode==2) {
        mn.p1_idx++;
        if ( mn.p1_idx>mn.p1_len) {
            mn.p1_idx=0;
			c++;
		}
		
        l=mn.p1_len;
        if ( l>16)
            l=16;
        Daisy24_LCD_SetCursor( 0, 0);
        for ( i=0; i<l;i++) {
			t=(i+mn.p1_idx)%mn.p1_len;
            Daisy24_LCD_WriteChar(*(mn.p1+t));
        }
        
        if ( i<16) {
            for ( ;i<16;i++)
                Daisy24_LCD_WriteChar(' ');
        }
        
        Daisy24_LCD_SetCursor( 0, 1);
        Daisy24_LCD_WriteString(mn.p2);
    }

	// mode 3 s1 fixed s2 looping
    if ( mn.mode==3) {
	    mn.p2_idx++;
	    if ( mn.p2_idx>mn.p2_len) {
			mn.p2_idx=0;
			c++;
	    }
		
	    l=mn.p2_len;
	    if ( l>16)
	    l=16;
	    Daisy24_LCD_SetCursor( 0, 1);
	    for ( i=0; i<l;i++) {
		    t=(i+mn.p2_idx)%mn.p2_len;
		    Daisy24_LCD_WriteChar(*(mn.p2+t));
	    }
	    
	    if ( i<16) {
		    for ( ;i<16;i++)
		    Daisy24_LCD_WriteChar(' ');
	    }
	    
	    Daisy24_LCD_SetCursor( 0, 0);
	    Daisy24_LCD_WriteString(mn.p1);
    }

	// mode 4 both string looping
	if ( mn.mode==4) {
        mn.p1_idx++;
        if ( mn.p1_idx>mn.p1_len) {
			mn.p1_idx=0;
			c++;
		}
        
        l=mn.p1_len;
        if ( l>16)
        l=16;
        Daisy24_LCD_SetCursor( 0, 0);
        for ( i=0; i<l;i++) {
	        t=(i+mn.p1_idx)%mn.p1_len;
	        Daisy24_LCD_WriteChar(*(mn.p1+t));
        }
        
        if ( i<16) {
	        for ( ;i<16;i++)
	        Daisy24_LCD_WriteChar(' ');
        }
        //
	    mn.p2_idx++;
	    if ( mn.p2_idx>mn.p2_len) {
			mn.p2_idx=0;
			c++;
	    }
		
	    l=mn.p2_len;
	    if ( l>16)
	    l=16;
	    Daisy24_LCD_SetCursor( 0, 1);
	    for ( i=0; i<l;i++) {
		    t=(i+mn.p1_idx)%mn.p2_len;
		    Daisy24_LCD_WriteChar(*(mn.p2+t));
	    }
	        
	    if ( i<16) {
		    for ( ;i<16;i++)
		    Daisy24_LCD_WriteChar(' ');
	    }	
	}
	
	// update the loop counter
	*f=c;
	
	return 0;
}

/**
 * \brief LCD Monitor Stop, stop the monitor activity
 *
 * \param[out]	f return the number of time the strings loop
 * \retval  0 OK, 1 Error
 * 
 */
uint32_t Daisy24_LCD_MonitorStop( void)
{
	mn.init==0;
	return 0;
}

/**
 * \brief Get the push button state
 * \param[out]	p the pushed button
 *
 *						returned bit  value
 *                -----+-------------+
 *                     |   1     2   |
 *                     |  +-+   +-+  |---+
 *                     |  +-+   +-+  |   |
 *                     |             | O |
 *                     |   8     4   |   |
 *                     |  +-+   +-+  |---+
 *                     |  +-+   +-+  |
 *                -----+-------------+ 
 * 
 * \retval 0 OK, 1 Error
 */
uint32_t Daisy24_PB_Status( uint32_t*p)
{
    uint8_t buff[1];
	uint8_t r;
    
    if ( Daisy24_Init_Done==0)
		return 1;
    
    buff[0]=0xFF;
    r=Daisy24_write_RegisterMultiValue( &i2c_master_instance, EXPANDER_I2C_ADDR, &buff[0], 1);
	if ( r != STATUS_OK) return r;
    r=Daisy24_read_RegisterMultiValue( &i2c_master_instance, EXPANDER_I2C_ADDR, 0xFF, &buff[0], 1);
    if ( r != STATUS_OK) return r;
	
	buff[0] &= 0x0F;
	*p = buff[0] ^ 0x0F; 
	
    return 0;	
}

/*
 *
*/
int32_t Daisy24_write_RegisterMultiValue( struct i2c_master_module*module, uint8_t i2c_addr, uint8_t*value, const uint16_t n)
{
	uint8_t r;
	
	packet.address     	= i2c_addr,
	packet.data_length 	= n,
	packet.data        	= value,
	packet.ten_bit_address = false,
	packet.high_speed      = false,
	packet.hs_master_code  = 0x0,

	r=i2c_master_write_packet_wait( module, &packet);

	return r;
}

/*
 *
*/
int32_t Daisy24_read_RegisterMultiValue( struct i2c_master_module*module, uint8_t i2c_addr, uint8_t reg, uint8_t *value, const uint16_t n)
{
	uint8_t tmpbuffer[1];
	uint8_t r;
	
	tmpbuffer[0] = reg;
	
	packet.address     	= i2c_addr,
	packet.data_length 	= 1,
	packet.data        	= tmpbuffer,
	packet.ten_bit_address = false,
	packet.high_speed      = false,
	packet.hs_master_code  = 0x0,

	r=i2c_master_write_packet_wait_no_stop( module, &packet);
	if ( r != STATUS_OK) return r;

	packet.address     	= i2c_addr,
	packet.data_length 	= n,
	packet.data        	= value,
	packet.ten_bit_address = false,
	packet.high_speed      = false,
	packet.hs_master_code  = 0x0,
	
	r=i2c_master_read_packet_wait( module, &packet);
	
	return r;
	
}


