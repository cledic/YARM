/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <extint.h>
#include <extint_callback.h>

#include <serial.h>
#include <stdio_serial.h>
#include <sam0_usart/usart_serial.h>

#include "YARM_lib.h"
#include "BME280_lib.h"
#include "LowPower_lib.h"
#include "terminal.h"

/* Serial interface: 115200,n,8,1
 * PA22 -> Tx
 * PA23 -> RX
*/
#define CONF_STDIO_USART          SERCOM5
#define CONF_STDIO_MUX_SETTING    USART_RX_1_TX_0_XCK_1
#define CONF_STDIO_PINMUX_PAD0    PINMUX_PA22D_SERCOM5_PAD0
#define CONF_STDIO_PINMUX_PAD1    PINMUX_PA23D_SERCOM5_PAD1
#define CONF_STDIO_PINMUX_PAD2    PINMUX_UNUSED
#define CONF_STDIO_PINMUX_PAD3    PINMUX_UNUSED
#define CONF_STDIO_BAUDRATE       115200

#define CONF_STDIO_PAD0_PIN PIN_PA22D_SERCOM5_PAD0
#define CONF_STDIO_PAD1_PIN PIN_PA23D_SERCOM5_PAD1

static struct usart_module usart_instance;
static void configure_usart(void);
/* ****************************************************** */

/* GPIO pin connected to ATA8510 Event pin.
*/
#define EVENT_PIN                   PIN_PA14
#define EVENT_ACTIVE                false
#define EVENT_INACTIVE              !EVENT_ACTIVE
#define EVENT_EIC_PIN               PIN_PA14A_EIC_EXTINT14
#define EVENT_EIC_MUX               MUX_PA14A_EIC_EXTINT14
#define EVENT_EIC_PINMUX            PINMUX_PA14A_EIC_EXTINT14
#define EVENT_EIC_LINE              14
//
void Event_ExtIntChannel(void);
void Event_ExtIntCallbacks(void);
void Event_Callback(void);
volatile uint32_t event_state;
/* ****************************************************** */

/* Data union to manage different format as array of chars
*/
typedef union
{
	uint8_t data[32];
	struct {
		float t;
		float p;
		float h;
		uint32_t SerialNumber[4];
	} wval;
} SENSOR_DATA_t;

SENSOR_DATA_t sd;
#define SENSOR_DATA_LEN	(sizeof(SENSOR_DATA_t))
/* ****************************************************** */

/* receive buffer used for all transactions */
#define BUF_LENGTH	64
static uint8_t rd_buffer[BUF_LENGTH];

int32_t	buffer_len;
float t, h, p;
uint8_t TxRxEvent[16];
uint8_t TxPreambleBuffer[]={0x04, 0x70, 0x8E, 0x0A, 0x55, 0x55, 0x10, 0x55, 0x56};
//
volatile uint32_t *ser_ptr1 = (volatile uint32_t *)0x0080A00C;
volatile uint32_t *ser_ptr2 = (volatile uint32_t *)0x0080A040;
uint8_t	serviceChannelReceived;
uint32_t rssiReceived;
uint32_t SerialNumber[4];
uint32_t counter_cycle;
uint32_t rxError;
uint32_t rxLength;
uint32_t txLength;
uint32_t i, ii;
volatile uint32_t ConsoleOn;

void PrintSysError( const char*s);
uint32_t Read_ReceivedData( void);

// http://atmel.force.com/support/articles/en_US/FAQ/Printf-with-floating-point-support-in-armgcc
// per visualizzare i float con la printf ho aggiunto
// in Properties,Toolchain, in -> ARM/GNU Linker, Miscellaneous
// -lc -u _printf_float
// nelle flags

int main (void)
{
	ConsoleOn=0;

	system_init();

	/* BOD33 disabled */
	SUPC->BOD33.reg &= ~SUPC_BOD33_ENABLE;

	/* VDDCORE is supplied BUCK converter */
	SUPC->VREG.bit.SEL = SUPC_VREG_SEL_BUCK_Val;

	configure_usart();

	Event_ExtIntChannel();
	Event_ExtIntCallbacks();
	system_interrupt_enable_global();
	
	if ( ConsoleOn) Term_Banner();

	for ( i=0; i<SENSOR_DATA_LEN; i++)
	sd.data[i]=0;

	/* */
	for ( i=0; i<BUF_LENGTH; i++) {
		rd_buffer[i]=0;
	}

	/* Configure SPI and PowerUp ATA8510 */
	YARM_Init();
	delay_ms(1);
	if ( ConsoleOn) printf("YARM_Init\r\n")	;
	
	/* MCU ID */
	SerialNumber[0]=*ser_ptr1;
	SerialNumber[1]=*ser_ptr2++;
	SerialNumber[2]=*ser_ptr2++;
	SerialNumber[3]=*ser_ptr2++;

	YARM_SetIdleMode();
	if ( ConsoleOn) PrintSysError("YARM_SetIdleMode");
	delay_ms(1);

	YARM_SetPollingMode();
	if ( ConsoleOn) PrintSysError("YARM_SetPollingMode");

	/* Ciclo principale */
	while( 1)
	{

#if 0
	#if 0
			//
			EnterSleepMode2();	// OK	
	#else
			// 
			EnterIdleMode();
	#endif
#endif	

		/* Event from ATA8510 */
		if ( event_state) {
			event_state=0;
			/* Read data from ATA8510 */
			Read_ReceivedData();
		}
	}

}

uint32_t Read_ReceivedData( void)
{
	//
	for ( i=0; i<BUF_LENGTH; i++) {
		rd_buffer[i]=0;
	}
	
	/* Start a loop to wait for the EOTA flag */
	rxError=0;
	ii=0;
	do {
		delay_us( 500);
		//
		buffer_len = YARM_GetEventBytes( &rd_buffer[0]);
		serviceChannelReceived = rd_buffer[3];
	} while( (rd_buffer[1] & 0x10) == 0);			// EOTA: End of Telegram on path A
	
	delay_us( 5000);
	
	// clear data buffer
	for ( i=0; i<SENSOR_DATA_LEN; i++) {
		sd.data[i]=0;
	}
	
	// verify Rx FIFO
	buffer_len = YARM_ReadRxFillLevel( &rd_buffer[0]);
	i=rd_buffer[0];
	// check FIFO length
	if ( i>0 && i <= 32) {
		YARM_ReadRxFifo( i, rd_buffer);
		// Checksum test
		if (rd_buffer[i-1+3] != checksum(rd_buffer+3, i-1)) {
			rxError|=1;
		}
	}
	
	//
	TERM_TEXT_GREEN;
	printf("-----------------------------------------------------\r\n");
	printf("Received %d bytes [%d]\r\n", i, rxError);
	TERM_TEXT_DEFAULT;
	
	// copy byte data on sensors data structure
	for ( ii=0; ii<SENSOR_DATA_LEN; ii++) {
		sd.data[ii]=rd_buffer[ii+3];
	}
	
	buffer_len = YARM_ReadRssiFillLevel( &rd_buffer[0]);
	
	rssiReceived=0;
	i=rd_buffer[0];
	
	if ( i!=0) {
		YARM_ReadRssiFifo( i, rd_buffer);
		ii=0;
		while( ii<i) {
			rssiReceived += rd_buffer[ii+3];
			ii++;
		}
		//
		rssiReceived/=i;
	} else {
		rxError |= 2;
	}
	
	// prepare the JSON string
	TERM_TEXT_GREEN;
	printf("remoteID: 0x%04X%04X%04X%04X\r\n",sd.wval.SerialNumber[0],
											sd.wval.SerialNumber[1],
											sd.wval.SerialNumber[2],
											sd.wval.SerialNumber[3]);
	printf("temperature: %.2f\r\n",sd.wval.t);
	printf("pressure: %.2f\r\n", (sd.wval.p/100.0f));
	printf("humidity: %.2f\r\n", sd.wval.h);
	printf("RSSI: %.2f dBm\r\n", (rssiReceived*1.0f/2.0f)-134.0f);
	printf("RxError: %d\r\n", rxError);
	printf("-----------------------------------------------------\r\n");
	TERM_TEXT_DEFAULT;
	printf("\r\n\r\n");

	YARM_SetPollingMode();
	if ( ConsoleOn) PrintSysError("YARM_SetPollingMode");

	return 0;
}
	
void Event_ExtIntChannel(void)
{
	struct extint_chan_conf config_extint_chan;
	extint_chan_get_config_defaults( &config_extint_chan);

	config_extint_chan.gpio_pin           = EVENT_EIC_PIN;
	config_extint_chan.gpio_pin_mux       = EVENT_EIC_MUX;
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_UP;
	config_extint_chan.detection_criteria = EXTINT_DETECT_FALLING;
	extint_chan_set_config( EVENT_EIC_LINE, &config_extint_chan);
}

void Event_ExtIntCallbacks(void)
{
	extint_register_callback( Event_Callback, EVENT_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback( EVENT_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT);
}

void Event_Callback(void)
{
	event_state = 1;
}

/**
 * \brief Configure usart and stdio.
 */
void configure_usart(void)
{
	struct usart_config config_usart;
	
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate    = CONF_STDIO_BAUDRATE;
	config_usart.mux_setting = CONF_STDIO_MUX_SETTING;
	config_usart.pinmux_pad0 = CONF_STDIO_PINMUX_PAD0;
	config_usart.pinmux_pad1 = CONF_STDIO_PINMUX_PAD1;
	config_usart.pinmux_pad2 = CONF_STDIO_PINMUX_PAD2;
	config_usart.pinmux_pad3 = CONF_STDIO_PINMUX_PAD3;
	
	while (usart_init(&usart_instance, CONF_STDIO_USART, &config_usart) != STATUS_OK); 
	stdio_serial_init(&usart_instance, CONF_STDIO_USART, &config_usart);

	usart_enable(&usart_instance);
}

void PrintSysError( const char*s)
{
	//TERM_CURSOR_SAVE;
	if ( YARM_GetSysError())
	{
		//
		YARM_ReadSram( 1, 0x03, 0x00, TxRxEvent);
		TERM_BKGRD_WHITE;
		TERM_TEXT_RED;
		//TERM_CURSOR_POS(1,1);
		printf("Error at %s, ErrorCode: %d\r\n", s, TxRxEvent[5]);
		} else {
		TERM_TEXT_DEFAULT;
		//TERM_CURSOR_POS(1,1);
		printf("Done %s [0x%X 0x%X]\r\n", s, YARM_Events[0], YARM_Events[1]);
	}
	TERM_TEXT_DEFAULT;
	//TERM_CURSOR_RESTORE;
}
