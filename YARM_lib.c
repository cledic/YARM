/************************************************************************/
/** 
 * \brief	Versione x Demo con L21E18B
 *			8510 ON/OFF
 *				PA14 /EVENT8510
 *				PA27 /NPWRON
 *				PA15 NRESET8510
 *			USART x stdio
 *				PA22
 *				PA23
 *			8510 SPI Master
 *				PA04D (PAD0)
 *				PA05D (PAD1)
 *				PA06D (PAD2)
 *				PA07D (PAD3)
 *
*/
/************************************************************************/
#include <compiler.h>
#include <board.h>
#include <conf_board.h>
#include <port.h>
#include <asf.h>
//
#include <YARM_lib.h>
#include "terminal.h"

#include <serial.h>
#include <stdio_serial.h>
#include <sam0_usart/usart_serial.h>

// PA14 /EVENT8510
#define EVENT8510_PIN               PIN_PA14
#define EVENT8510_ACTIVE            false
#define EVENT8510_INACTIVE          !EVENT8510_ACTIVE
// PA27 /NPWRON
#define NPWRON_PIN                  PIN_PA27
#define NPWRON_ACTIVE               false
#define NPWRON_INACTIVE             !NPWRON_ACTIVE
// PA15 NRESET8510
#define NRESET8510_PIN              PIN_PA15
#define NRESET8510_ACTIVE           false
#define NRESET8510_INACTIVE         !NRESET8510_ACTIVE

#if 0
#define CONF_STDIO_USART          SERCOM3
#define CONF_STDIO_MUX_SETTING    USART_RX_1_TX_0_XCK_1
#define CONF_STDIO_PINMUX_PAD0    PINMUX_PA22C_SERCOM3_PAD0
#define CONF_STDIO_PINMUX_PAD1    PINMUX_PA23C_SERCOM3_PAD1
#define CONF_STDIO_PINMUX_PAD2    PINMUX_UNUSED
#define CONF_STDIO_PINMUX_PAD3    PINMUX_UNUSED
#define CONF_STDIO_BAUDRATE       38400

#define CONF_STDIO_PAD0_PIN PIN_PA22C_SERCOM3_PAD0
#define CONF_STDIO_PAD1_PIN PIN_PA23C_SERCOM3_PAD1

static struct usart_module usart_instance;
#endif

uint8_t SPI_Delay[]={
	0,			// 0x00
	0,			// 0x01 Read Fill Level Rx FIFO
	0,			// 0x02 Read Fill Level Tx FIFO
	0,			// 0x03 Read Fill Level RSSI FIFO
	0,			// 0x04 Get Event Bytes
	120,		// 0x05 Read RSSI FIFO
	120,		// 0x06 Read Rx FIFO
	110,		// 0x07 Write SRAM Reg
	120,		// 0x08 Read SRAM Reg
	55,			// 0x09 Write EEPROM
	0,			// 0x0A Read EEPROM
	110,		// 0x0B Write Tx FIFO
	110,		// 0x0C Write Tx Preamble FIFO
	55,			// 0x0D Set System Mode
	50,			// 0x0E Calibrate and Check
	0,			// 0x0F Patch SPI
	0,			// 0x10 Get RESET ROM
	0,			// 0x11
	0,			// 0x12
	0,			// 0x13 Get Version Flash
	0,			// 0x14 Customer Configurable Command
	0,			// 0x15 Systems RESET
	65,			// 0x16 Trigger EEPROM Secure Write
	85,			// 0x17 Set Voltage Monitor
	0,			// 0x18 OFF Command
	0,			// 0x19 Read Temperature Value
	50,			// 0x1A Init SRAM Service
	55,			// 0x1B Start RSSI Measurement
	0,			// 0x1C Get RSSI Value
	70,			// 0x1D Read Rx FIFO Byte Interrupt
	70,			// 0x1E Read RSSI FIFO Byte Interrupt
};

//#define RXBUFFER_LEN	32
//uint8_t	rxbuff[RXBUFFER_LEN];
//
//#define CMD_GETVERSIONFLASH_LEN	6
//static uint8_t CmdGetVersionFlash_buffer[CMD_GETVERSIONFLASH_LEN] = {
//	0x13, 0x00, 0x00, 0x00, 0x00, 0x00,
//};
//
//#define CMD_SETIDLEMODE_LEN	3
//static uint8_t CmdSetIdleMode_buffer[CMD_SETIDLEMODE_LEN] = {
//	0x0D, 0x20, 0x00,
//};
//
//#define CMD_READTEMPERATURE_LEN	4
//static uint8_t CmdReadTemperature_buffer[CMD_READTEMPERATURE_LEN] = {
//	0x19, 0x00, 0x00, 0x00,
//};
//
//#define CMD_GETEVENTBYTES_LEN	4
//static uint8_t CmdGetEventBytes_buffer[CMD_GETEVENTBYTES_LEN] = {
//	0x04, 0x00, 0x00, 0x00,
//};

static int32_t	YARM_InitDone = false;
YARM_State_t	YARM_CurrState = YARM_IDLE;
YARM_Cmd_t		YARM_Cmd = GETVERSIONFLASH;

uint8_t YARM_Events[2];
uint8_t SysError;
uint8_t YARM_Verbose = 0;

struct spi_module spi_master_instance;
struct spi_slave_inst slave;
status_code_genare_t spi_ret;
volatile bool transrev_complete_spi_master = false;

/* Private function */
//static void configure_usart(void);
void configure_spi_master_callbacks(void);
void configure_spi_master(void);
unsigned char checksum (unsigned char *ptr, size_t sz);
/* ************************************************************************ */

static void callback_spi_master( struct spi_module *const module)
{
	transrev_complete_spi_master = true;
}


void configure_spi_master_callbacks(void)
{
	spi_register_callback(&spi_master_instance, callback_spi_master, SPI_CALLBACK_BUFFER_TRANSCEIVED);
	spi_enable_callback(&spi_master_instance, SPI_CALLBACK_BUFFER_TRANSCEIVED);
}

#define YARM_SPI_MODULE              SERCOM0
#define YARM_SPI_SERCOM_MUX_SETTING  SPI_SIGNAL_MUX_SETTING_E
#define YARM_SPI_SERCOM_PINMUX_PAD0  PINMUX_PA04D_SERCOM0_PAD0
#define YARM_SPI_SERCOM_PINMUX_PAD1  PINMUX_PA05D_SERCOM0_PAD1
#define YARM_SPI_SERCOM_PINMUX_PAD2  PINMUX_PA06D_SERCOM0_PAD2
#define YARM_SPI_SERCOM_PINMUX_PAD3  PINMUX_PA07D_SERCOM0_PAD3
#define YARM_SPI_SELECT_PIN PIN_PA05

//#define EXT1_SPI_SERCOM_DMAC_ID_TX   SERCOM0_DMAC_ID_TX
//#define EXT1_SPI_SERCOM_DMAC_ID_RX   SERCOM0_DMAC_ID_RX

void configure_spi_master(void)
{

	struct spi_config config_spi_master;
	struct spi_slave_inst_config slave_dev_config;
	/* Configure and initialize software device instance of peripheral slave */
	spi_slave_inst_get_config_defaults(&slave_dev_config);
	slave_dev_config.ss_pin = YARM_SPI_SELECT_PIN;
	spi_attach_slave(&slave, &slave_dev_config);
	/* Configure, initialize and enable SERCOM SPI module */
	spi_get_config_defaults(&config_spi_master);
	config_spi_master.mux_setting = YARM_SPI_SERCOM_MUX_SETTING;
	/* Configure pad 0 for data in */
	config_spi_master.pinmux_pad0 = YARM_SPI_SERCOM_PINMUX_PAD0;
	/* Configure pad 1 as unused */
	config_spi_master.pinmux_pad1 = PINMUX_UNUSED;
	/* Configure pad 2 for data out */
	config_spi_master.pinmux_pad2 = YARM_SPI_SERCOM_PINMUX_PAD2;
	/* Configure pad 3 for SCK */
	config_spi_master.pinmux_pad3 = YARM_SPI_SERCOM_PINMUX_PAD3;
	spi_init(&spi_master_instance, YARM_SPI_MODULE, &config_spi_master);

	spi_enable(&spi_master_instance);

}

void YARM_SPIDisable( void)
{
	spi_disable( &spi_master_instance);
}

void YARM_SPIEnable( void)
{
	spi_enable( &spi_master_instance);
}

int32_t YARM_Init( void)
{
	struct port_config pin_conf;
	
	// Configure printf through serial port
	//configure_usart();
	
	port_get_config_defaults(&pin_conf);

	/*
	 *		PA14 /EVENT8510	(Inout)
	 *		PA27 /NPWRON	(Output)
	 *		PA15 NRESET8510	(Output)	
	*/
	/* Do not initialize the EVENT pin if you are configuring the External Interrupt */
#if 0
	/* Set EVENT8510 as inputs */
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(EVENT8510_PIN, &pin_conf);
#endif

	/* Configure /NPWRON as outputs, turn active */
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_output_level(NPWRON_PIN, NPWRON_INACTIVE);
	port_pin_set_config(NPWRON_PIN, &pin_conf);
	

	/* Configure NRESET8510 as outputs, turn active */
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_output_level(NRESET8510_PIN, NRESET8510_INACTIVE);
	port_pin_set_config(NRESET8510_PIN, &pin_conf);

	// Configure SPI master and callback
	configure_spi_master();
	configure_spi_master_callbacks();
	
	if ( YARM_Verbose)
		printf("YARM Init Done...\r\n");
	
	YARM_PowerOn();
	
	return 0;
}

int32_t YARM_PowerOn( void)
{
	port_pin_set_output_level(NRESET8510_PIN, NRESET8510_ACTIVE);
	port_pin_set_output_level(NRESET8510_PIN, NRESET8510_ACTIVE);
	port_pin_set_output_level(NRESET8510_PIN, NRESET8510_ACTIVE);
	port_pin_set_output_level(NRESET8510_PIN, NRESET8510_ACTIVE);
	port_pin_set_output_level(NRESET8510_PIN, NRESET8510_INACTIVE);
	port_pin_set_output_level(NRESET8510_PIN, NRESET8510_INACTIVE);
	port_pin_set_output_level(NRESET8510_PIN, NRESET8510_INACTIVE);
	port_pin_set_output_level(NRESET8510_PIN, NRESET8510_INACTIVE);
	port_pin_set_output_level(NPWRON_PIN, NPWRON_ACTIVE);
	
	delay_ms(2);
	
	if ( YARM_Verbose)
		printf("YARM PowerOn Done...\r\n");
	YARM_InitDone=true;
	
	return 0;
}

int32_t YARM_PowerOff( void)
{
	port_pin_set_output_level(NPWRON_PIN, NPWRON_INACTIVE);			// true
	port_pin_set_output_level(NRESET8510_PIN, NRESET8510_ACTIVE);	// false
	port_pin_set_output_level(NRESET8510_PIN, NRESET8510_INACTIVE /*NRESET8510_ACTIVE*/);	// false
	
	return 0;
}

int32_t YARM_GetInitDone( void)
{
	return YARM_InitDone;
}

int32_t YARM_GetEvent( void)
{
	return port_pin_get_input_level( EVENT8510_PIN);
}

/**
 * \brief Switch to polling mode of the RF receiver
 *
 * Switch to polling mode with VCO tuning enabled and starting with polling configuration 0.
 */
int32_t YARM_SetPollingMode( void)
{
	int32_t ret;
	// set polling mode 
	uint8_t command[3]={ YARM_CMD_SETSYSTEMMODE, YARM_CMD_SETPOLLINGMODE, 0x00};
	uint8_t dummy[3];

	ret=YARM_SendCmd( command, dummy, YARM_CMD_SETPOLLINGMODE_LEN);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,dummy,3);
	//set_ss_inactive();
	
	return ret;
}


/**
 * \brief Switch to idle mode of the RF receiver
 *
 * Switch to idle mode.
 */
int32_t YARM_SetIdleMode(void)
{
	int32_t ret;
	
	// set idle mode
	uint8_t command[3]={ YARM_CMD_SETSYSTEMMODE, YARM_RF_IDLEMODE, YARM_RF_RXSERVICE};
	uint8_t dummy[3];
	
	ret=YARM_SendCmd( command, dummy, YARM_CMD_SETIDLEMODE_LEN);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,dummy,3);
	//set_ss_inactive();
	
	return ret;
}

/**
 * \brief Read the fill level of the RX buffer of the RF receiver
 *
 * Read the buffer level of the receiver data buffer.
 */
int32_t YARM_ReadRxFillLevel( uint8_t*d)
{	
	int32_t ret;
	
	uint8_t command[3]={ YARM_CMD_READRXFILLLEVEL, 0x00, 0x00};
	uint8_t data[3];
	
	ret=YARM_SendCmd( command, data, YARM_CMD_READRXFILLLEVEL_LEN);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,data,3);
	//set_ss_inactive();

	// fill level is in last received data byte 
	*d=data[2];
	
	return ret;
}

/**
 * \brief Read the fill level of the TX buffer of the RF receiver
 *
 * Read the buffer level of the transmitter buffer.
 */
int32_t YARM_ReadTxFillLevel( uint8_t*d)
{	
	int32_t ret;
	
	uint8_t command[3]={YARM_CMD_READTXFILLLEVEL,0x00,0x00};
	uint8_t data[3];
	
	ret=YARM_SendCmd( command, data, YARM_CMD_READTXFILLLEVEL_LEN);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,data,3);
	//set_ss_inactive();

	// fill level is in last received data byte 
	*d=data[2];
	
	return ret;
}


/**
 * \brief Read the fill level of the RSSI buffer of the RF receiver
 *
 * Read the buffer level of the RSSI buffer.
 */
int32_t YARM_ReadRssiFillLevel( uint8_t*d)
{	
	int32_t ret;
	
	uint8_t command[3]={YARM_CMD_READRSSIFILLLEVEL, 0x00, 0x00};
	uint8_t data[3];
	
	ret=YARM_SendCmd( command, data, YARM_CMD_READTXFILLLEVEL_LEN);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,data,3);
	//set_ss_inactive();

	// fill level is in last received data byte 
	*d=data[2];
	
	return ret;
}

/**
 * \brief Start the RSSI measurement
 *
 * 
 */
int32_t YARM_StartRSSIMeasurement( void)
{	
	int32_t ret;
	
	uint8_t command[2]={YARM_CMD_STARTRSSIMSRMNT, 0x00};
	uint8_t data[2];
	
	ret=YARM_SendCmd( command, data, YARM_CMD_STARTRSSIMSRMNT_LEN);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,data,3);
	//set_ss_inactive();
	
	return ret;
}

/**
 * \brief Get the RSSI measurement
 *
 * 
 */
int32_t YARM_GetRSSIMeasurement( uint8_t*d)
{	
	int32_t ret;
	
	uint8_t command[4]={YARM_CMD_GETRSSIMSRMNT, 0x00, 0x00, 0x00};
	uint8_t data[4];
	
	ret=YARM_SendCmd( command, data, YARM_CMD_GETRSSIMSRMNT_LEN);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,data,3);
	//set_ss_inactive();
	
	d[0]=data[2];	// Last measured RSSI average value
	d[1]=data[3];	// Last measured RSSI peak value
	return ret;
}

/**
 * \brief Read the config event byte of the RF receiver
 *
 * Read the event bytes of the receiver and return the config event byte.
 */
uint8_t YARM_GetConfigEventByte( uint8_t*d)
{
	int32_t ret;
	uint8_t command[4]={YARM_CMD_GETCONFIGEVENTBYTE,0x00,0x00,0x00};
	uint8_t data[4];
	
	ret=YARM_SendCmd( command, data, YARM_CMD_GETCONFIGEVENTBYTE_LEN);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,data,4);
	//set_ss_inactive();

	// return config event byte with channel, service and path information
	*d=data[3];
	
	return ret;
}

/**
 * \brief Read the event bytes of the RF receiver
 *
 * Read the event bytes of the receiver and return these bytes.
 */
int32_t YARM_GetEventBytes(uint8_t *data)
{
	int32_t ret;
	uint8_t command[4]={YARM_CMD_GETEVENTBYTE, 0x00, 0x00, 0x00};
	
	ret=YARM_SendCmd( command, data, YARM_CMD_GETEVENTBYTE_LEN);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,data,4);
	//set_ss_inactive();
	
	return ret;

}

/**
 * \brief Read RSSI buffer of the RF receiver
 *
 * Read the RSSI buffer of the receiver.
 */
int32_t YARM_ReadRssiFifo(uint8_t data_size, uint8_t *data)
{
	int32_t ret;
	uint8_t command[35]={YARM_CMD_READRSSIFIFO,data_size};
	
	ret=YARM_SendCmd( command, data, data_size+3);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,data,data_size+3);
	//set_ss_inactive();

	return ret;
}

/**
 * \brief Read RX buffer of the RF receiver
 *
 * Read the RX buffer of the receiver.
 */
int32_t YARM_ReadRxFifo(uint8_t data_size, uint8_t *data)
{
	int32_t ret;
	uint8_t command[35]={YARM_CMD_READRXFIFO,data_size};
	
	ret=YARM_SendCmd( command, data, data_size+3);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,data,data_size+3);
	//set_ss_inactive();
	
	return ret;

}
/**
 * \brief Write SRAM/REG of the RF receiver
 *
 * Write content to SRAM/Reg of the receiver.
 */
int32_t YARM_WriteSram( uint8_t data_size, uint8_t addr_h, uint8_t addr_l, uint8_t *data)
{
	int32_t ret;
	uint8_t command[35]={YARM_CMD_WRITESRAM, data_size, addr_h, addr_l};
	uint8_t index;
	
	for ( index=0; index < data_size; index++)
	{
		command[4 + index]= data[index];
	}
	
	ret=YARM_SendCmd( command, data, data_size+YARM_CMD_WRITESRAM_LEN);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,data,data_size+4);
	//set_ss_inactive();

	return ret;
}


/**
 * \brief Read SRAM/REG of the RF receiver
 *
 * Read content from SRAM/Reg of the receiver.
 */
int32_t YARM_ReadSram(uint8_t data_size, uint8_t addr_h, uint8_t addr_l, uint8_t *data)
{
	int32_t ret;
	uint8_t command[35]={YARM_CMD_READSRAM, data_size, addr_h, addr_l, 0x00};
	
	ret=YARM_SendCmd( command, data, data_size+YARM_CMD_READSRAM_LEN);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,data,data_size+5);
	//set_ss_inactive();

	return ret;
}

/**
 * \brief Write byte to EEPROM
 *
 * Write one byte to the EEPROM of the receiver.
 */
int32_t YARM_WriteEeprom(uint8_t ee_addr_h, uint8_t ee_addr_l, uint8_t data)
{
	int32_t ret;
	uint8_t command[4]={YARM_CMD_WRITEEEPROM,ee_addr_h, ee_addr_l, data};
	uint8_t dummy[4];
	
	ret=YARM_SendCmd( command, dummy, YARM_CMD_WRITEEEPROM_LEN);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,dummy,4);
	//set_ss_inactive();

	return ret;
}

/**
 * \brief Read byte from EEPROM
 *
 * Read one byte from the EEPROM of the receiver.
 */
int32_t YARM_ReadEeprom(uint8_t ee_addr_h, uint8_t ee_addr_l, uint8_t*d)
{
	int32_t ret;
	uint8_t command[5]={YARM_CMD_READEEPROM, ee_addr_h, ee_addr_l, 0x00, 0x00};
	uint8_t data[5];
	
	ret=YARM_SendCmd( command, data, YARM_CMD_READEEPROM_LEN);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,data,5);
	//set_ss_inactive();
	
	*d=data[5];
	return ret;
}

/**
 * \brief Write TX buffer of the RF receiver
 *
 * Write data to the TX buffer of the receiver.
 */
int32_t YARM_WriteTxFifo( uint8_t data_size, uint8_t *data)
{
	int32_t ret;
	uint8_t command[35]={YARM_CMD_WRITETXFIFO, data_size};
	uint8_t dummy[35];
	uint8_t index;
	
	for (index=0; index <data_size; index++)
	{
		command[2 + index]= data[index];
	}
	
	ret=YARM_SendCmd( command, dummy, data_size+2);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,dummy,data_size+2);
	//set_ss_inactive();
	
	return ret;

}

/**
 * \brief Write TX preamble buffer of the RF receiver
 *
 * Write data to the TX preamble buffer of the receiver.
 */
int32_t YARM_WriteTxPreamble(uint8_t data_size, uint8_t *data)
{
	int32_t ret;
	uint8_t command[35]={YARM_CMD_WRITETXPREAMBLE, data_size};
	uint8_t dummy[35];
	uint8_t index;
	
	for (index=0; index <data_size; index++)
	{
		command[2 + index]= data[index];
	}
	
	ret=YARM_SendCmd( command, dummy, data_size+2);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,dummy,data_size+2);
	//set_ss_inactive();

	return ret;
}

/**
 * \brief Set system mode of the RF receiver
 *
 * Switch the transceiver to the wanted system mode.
 */
int32_t YARM_SetSystemMode(uint8_t mode_config, uint8_t service_channel)
{
	int32_t ret;
	uint8_t command[3]={YARM_CMD_SETSYSTEMMODE, mode_config, service_channel};
	uint8_t dummy[3];
	
	ret=YARM_SendCmd( command, dummy, YARM_CMD_SETSYSTEMMODE_LEN);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,dummy,3);
	//set_ss_inactive();
	
	return ret;
}

/**
 * \brief Read the ROM, Flash and customer version of the RF receiver
 *
 * Read the ROM, Flash and customer version of the receiver and return the ROM version.
 */
int32_t YARM_GetVersionFlash( uint8_t*d)
{
	int32_t ret;
	uint8_t command[6]={YARM_CMD_GETVERSIONFLASH,0x00,0x00,0x00,0x00,0x00};
	uint8_t data[6];
	
	ret=YARM_SendCmd( command, data, YARM_CMD_GETVERSIONFLASH_LEN);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,data,6);
	//set_ss_inactive();
	
	// return the ROM version code
	*d=data[2];
	return ret;
}

/**
 * \brief Perform a reset of the RF receiver
 *
 * Perform a reset of the RF receiver.
 */
int32_t YARM_Reset(void)
{
	int32_t ret;
	uint8_t command[2]={YARM_CMD_RESET, 0x00};
	uint8_t data[6];
	
	ret=YARM_SendCmd( command, data, YARM_CMD_RESET_LEN);
	//set_ss_active();
	//spi_transceiver_buffer_wait(&rf_master,command,data,2);
	//set_ss_inactive();
	
	return ret;
}

uint32_t YARM_PrepareTxData( uint8_t*b, uint8_t*d, uint32_t l)
{
	uint32_t i;
	
	b[0]=YARM_DATA_HEADER;
	for ( i=0; i<l; i++) {
		b[i+YARM_DATA_HEADER_LEN]=d[i];
	}
	i+=YARM_DATA_HEADER_LEN;
	b[i]=checksum( b, i);
	
	return l+YARM_DATA_HEADER_LEN+YARM_CHECKSUM_LEN;
}

/**
 * \brief Get the Sys Error last result
 *
 * 
 */
int32_t YARM_GetSysError( void)
{	
	return (int32_t)SysError;
}

unsigned char checksum (unsigned char *ptr, size_t sz)
{
	unsigned char chk = 0;

	while (sz-- != 0)
		chk -= *ptr++;

	return chk;
}

int32_t YARM_SendCmd( uint8_t*txb, uint8_t*rxb, uint8_t len)
{
	uint32_t tmout;
	
//	uint8_t *ptr;
//	uint32_t len;
//	
//	switch( cmd) {
//		case GETVERSIONFLASH:
//			ptr = &CmdGetVersionFlash_buffer[0];
//			len = CMD_GETVERSIONFLASH_LEN;
//		break;
//		case SETIDLEMODE:
//			ptr = &CmdSetIdleMode_buffer[0];
//			len = CMD_SETIDLEMODE_LEN;
//		break;
//		case READTEMPERATURE:
//			ptr=&CmdReadTemperature_buffer[0];
//			len = CMD_READTEMPERATURE_LEN;
//		break;
//		case GETEVENTBYTES:
//			ptr = &CmdGetEventBytes_buffer[0];
//			len = CMD_GETEVENTBYTES_LEN;
//		break;
//		default:
//			ptr = &CmdSetIdleMode_buffer[0];
//			len = CMD_SETIDLEMODE_LEN;		
//		break;
//	}
	
	spi_select_slave(&spi_master_instance, &slave, true);
	delay_us( 45);
	
	spi_ret=spi_transceive_buffer_job(&spi_master_instance, txb, rxb, len);
	
	//
	if ( spi_ret != 0) {
		// Error
		spi_select_slave(&spi_master_instance, &slave, false);
		return -spi_ret;
	}
	
	tmout=0;
	while (!transrev_complete_spi_master) {
		/////* Wait for write and read complete */
		delay_us(1);
		tmout++;
		if ( tmout>3000) {
			// Error...
			spi_select_slave(&spi_master_instance, &slave, false);
			return -STATUS_ERR_TIMEOUT;
		}
			
	}
	transrev_complete_spi_master = false;
	if ( SPI_Delay[ txb[0]])
		delay_us( SPI_Delay[ txb[0]]);
	spi_select_slave(&spi_master_instance, &slave, false);
	
	YARM_Events[0] = rxb[0];
	YARM_Events[1] = rxb[1];
	SysError=YARM_Events[0]&0x80;

	//spi_select_slave(&spi_master_instance, &slave, false);
	
	return len;
}

#if 0
/**
 * \brief Configure usart.
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
#endif
