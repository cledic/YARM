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

//
#define WEATHER_TX_CHNL		0x40		// Chn 1, Serv 0

//
void PrintSysError( const char*s);
uint32_t Transmit_Data( void);	
uint32_t PrepareTxData( void);

//
void rtc_overflow_callback(void);
void configure_rtc_count(void);
void configure_rtc_callbacks(void);
struct rtc_module rtc_instance;
volatile uint32_t event_rtc;

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

	/* Configure and enable RTC */
	configure_rtc_count();
	/* Configure and enable callback */
	configure_rtc_callbacks();
	/* Set period */
	rtc_count_set_period( &rtc_instance, 8000);

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
	if ( 1 /*ConsoleOn*/) printf("YARM_Init\r\n")	;
	
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

	counter_cycle=1;
	
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

		/* Evento dal modulo RTC */
		if ( event_rtc) {
			event_rtc=0;
			/* Trasmetto i dati */
			Transmit_Data();
		}
	}

}

/** Gestione della trasmissione di un msg via radio
 *
*/
uint32_t Transmit_Data( void)
{
	float alt=0.0f;
	uint32_t i;
	
	YARM_SetIdleMode();
	if ( ConsoleOn) PrintSysError("YARM_SetIdleMode");

#if 0	
	/* print out YARM state */
	buffer_len = YARM_GetEventBytes( &rd_buffer[0]);
	if ( ConsoleOn) {
		if ( buffer_len < 0 ) {
			printf( "Errore YARM_GetEventBytes %d\r\n", (buffer_len*-1));
			} else {
			printf( "Reg. GETEVENTBYTES: ");
			for ( i=0; i<buffer_len; i++) {
				printf( "0x%X, ", rd_buffer[i]);
			}
			printf( "\r\n");
		}
	}
	
	buffer_len = YARM_GetVersionFlash( &rd_buffer[0]);
	if ( ConsoleOn) {
		if ( buffer_len < 0 ) {
			printf( "Errore YARM_GetVersionFlash %d\r\n", (buffer_len*-1));
			} else {
			printf( "Reg. GETVERSIONFLASH: 0x%X\r\n", rd_buffer[0]);
		}
	}
	
	/* print out YARM state */
	buffer_len = YARM_GetEventBytes( &rd_buffer[0]);
	if ( ConsoleOn) {
		if ( buffer_len < 0 ) {
			printf( "Errore YARM_GetEventBytes %d\r\n", (buffer_len*-1));
			} else {
			printf( "Reg. GETEVENTBYTES: ");
			for ( i=0; i<buffer_len; i++) {
				printf( "0x%X, ", rd_buffer[i]);
			}
			printf( "\r\n");
		}
	}
#endif

	txLength = PrepareTxData();

	if (txLength==0)
		return 1;
	
	if ( ConsoleOn) {
		printf("Data to Send [%d]\r\n", txLength);
		for ( i=0; i<txLength;i++)
			printf(" 0x%0X, ", sd.data[i]);
	
		printf("\r\n");	
	}
	
	YARM_SetIdleMode(); 
	if ( ConsoleOn) PrintSysError("YARM_SetIdleMode");
	delay_ms(1);
	
	if ( 1 /*ConsoleOn*/) {
		TERM_TEXT_GREEN;
		printf("-------- START Transmission [%d]\r\n", counter_cycle++);
		TERM_TEXT_DEFAULT;
	}
	
	YARM_SetSystemMode( YARM_RF_TXMODE, WEATHER_TX_CHNL);
	if ( ConsoleOn) PrintSysError("YARM_SetSystemMode TXMode");
	delay_ms(1);
	
	YARM_WriteTxPreamble( YARM_WriteTxPreambleBuffer_LEN, &TxPreambleBuffer[0]);
	if ( ConsoleOn) PrintSysError("YARM_WriteTxPreamble");
	
	delay_us( 100);

	YARM_WriteTxFifo( txLength, sd.data);
	if ( ConsoleOn) PrintSysError("YARM_WriteTxFifo");
	delay_ms(100);
	
	if ( ConsoleOn) printf("TXLoop Started\r\n");
		
	// Controlla EVENT_IRQ
	event_state = 0;
	while( !event_state);
	event_state = 0;
	
	//
	i = 0;
	do {
		i++;
		delay_us( 3000);
		YARM_GetEventBytes( TxRxEvent);
		// PrintSysError( "YARM_GetEventBytes TXLoop");
	} while (((TxRxEvent[1] & 0x10) == 0)&&(i <40));

	if ( 1 /*ConsoleOn*/) {
		TERM_TEXT_GREEN;
		printf("-------- END Transmission \r\n");
		TERM_TEXT_DEFAULT;
	}
	
	if ( ConsoleOn) {
		printf("TXLoop Ended at cycle : %d [%d ms]\r\n", i, i*3);
		printf("TX Ended on service: %d channel: %d\r\n", (WEATHER_TX_CHNL & 0x07), (WEATHER_TX_CHNL>>4)&0x03);
	
		/* print out YARM state */
		buffer_len = YARM_GetEventBytes( &rd_buffer[0]);
		if ( buffer_len < 0 ) {
			printf( "Error YARM_GetEventBytes %d\r\n", (buffer_len*-1));
		} else {
			printf( "Reg. GETEVENTBYTES: ");
			for ( i=0; i<buffer_len; i++) {
				printf( "0x%X, ", rd_buffer[i]);
			}
			printf( "\r\n");
		}
	}
	
	/* */
	for ( i=0; i<SENSOR_DATA_LEN; i++) {
		sd.data[i]=0;
	}
	
	delay_us( 100);
	
	YARM_SetIdleMode(); 
	if ( ConsoleOn) PrintSysError("YARM_SetIdleMode");
	delay_ms( 1);
	
	return 1;
}

uint32_t PrepareTxData( void)
{
	uint32_t c;
	
	for ( c=0; c<SENSOR_DATA_LEN; c++) {
		sd.data[c]=0;
	}
		
	sd.wval.p = 3.4f;
	sd.wval.t = counter_cycle*1.0f;
	sd.wval.h = 5.6f;
	/* MCU ID */
	sd.wval.SerialNumber[0]=SerialNumber[0];
	sd.wval.SerialNumber[1]=SerialNumber[1];
	sd.wval.SerialNumber[2]=SerialNumber[2];
	sd.wval.SerialNumber[3]=SerialNumber[3];

	//
	sd.data[31]=checksum( &sd.data[0], 31);
	
	return 32;
}

void rtc_overflow_callback(void)
{
	/* Do something on RTC overflow here */
	event_rtc=1;
}

void configure_rtc_count(void)
{
	struct rtc_count_config config_rtc_count;
	rtc_count_get_config_defaults(&config_rtc_count);
	config_rtc_count.prescaler           = RTC_COUNT_PRESCALER_DIV_1;
	config_rtc_count.mode                = RTC_COUNT_MODE_16BIT;
	//	#ifdef FEATURE_RTC_CONTINUOUSLY_UPDATED
	//	config_rtc_count.continuously_update = true;
	//	#endif
	rtc_count_init(&rtc_instance, RTC, &config_rtc_count);
	rtc_count_enable(&rtc_instance);
}

void configure_rtc_callbacks(void)
{
	rtc_count_register_callback( &rtc_instance, rtc_overflow_callback, RTC_COUNT_CALLBACK_OVERFLOW);
	rtc_count_enable_callback( &rtc_instance, RTC_COUNT_CALLBACK_OVERFLOW);
	event_rtc=0;
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
