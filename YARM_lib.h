
#ifndef YARM_H_INCLUDED
#define YARM_H_INCLUDED

/**
 *
 */
#define YARM_CMD_SETPOLLINGMODE				(0x23)
#define YARM_CMD_SETIDLEMODE				(0x00)
#define YARM_CMD_READRXFILLLEVEL			(0x01)
#define YARM_CMD_READTXFILLLEVEL			(0x02)
#define YARM_CMD_READRSSIFILLLEVEL			(0x03)
#define YARM_CMD_GETCONFIGEVENTBYTE			(0x04)
#define YARM_CMD_GETEVENTBYTE				(0x04)	
#define YARM_CMD_READRSSIFIFO				(0x05)
#define YARM_CMD_READRXFIFO					(0x06)
#define YARM_CMD_WRITESRAM					(0x07)
#define YARM_CMD_READSRAM					(0x08)
#define YARM_CMD_WRITEEEPROM				(0x09)
#define YARM_CMD_READEEPROM					(0x0A)
#define YARM_CMD_WRITETXFIFO				(0x0B)
#define YARM_CMD_WRITETXPREAMBLE			(0x0C)
#define YARM_CMD_SETSYSTEMMODE				(0x0D)
#define YARM_CMD_GETVERSIONFLASH			(0x13)
#define YARM_CMD_RESET						(0x15)
#define YARM_CMD_STARTRSSIMSRMNT			(0x1B)
#define YARM_CMD_GETRSSIMSRMNT				(0x1C)

/**	
 */	
#define YARM_CMD_SETPOLLINGMODE_LEN			(3)
#define YARM_CMD_SETIDLEMODE_LEN			(3)
#define YARM_CMD_READRXFILLLEVEL_LEN		(3)
#define YARM_CMD_READTXFILLLEVEL_LEN		(3)
#define YARM_CMD_READRSSIFILLLEVEL_LEN		(3)
#define YARM_CMD_GETCONFIGEVENTBYTE_LEN		(4)
#define YARM_CMD_GETEVENTBYTE_LEN			(4)
#define YARM_CMD_WRITEEEPROM_LEN			(4)
#define YARM_CMD_READEEPROM_LEN				(5)
#define YARM_CMD_SETSYSTEMMODE_LEN			(3)
#define YARM_CMD_GETVERSIONFLASH_LEN		(6)
#define YARM_CMD_RESET_LEN					(2)
#define YARM_CMD_STARTRSSIMSRMNT_LEN		(2)
#define YARM_CMD_GETRSSIMSRMNT_LEN			(4)
#define YARM_CMD_WRITESRAM_LEN				(4)
#define YARM_CMD_READSRAM_LEN				(5)

#define YARM_DATA_HEADER					('D')
#define YARM_DATA_HEADER_LEN				(1)
#define YARM_CHECKSUM_LEN					(1)
#define YARM_WriteTxPreambleBuffer_LEN		(9)

													// systemModeConfig:
#define YARM_RF_RXPOLLINGMODE				(0x23)	// bit5 VCO tuning before changing OPM enabled,
													// OPM[3] RXPollingMode

													// systemModeConfig:
#define YARM_RF_IDLEMODE					(0x20)	// bit5 VCO tuning before changing OPM enabled,
													// OPM[0] IDLEMode

													// systemModeConfig:
#define YARM_RF_TXMODE						(0x21)	// bit5 VCO tuning before changing OPM enabled,
													// OPM[1] TXMode
													
													// systemModeConfig:													
#define YARM_RF_RXMODE						(0x22)  // bit5 VCO tuning before changing OPM enabled,
													// OPM[2] RXMode
													
													// serviceChannelConfig:													
#define YARM_RF_TXSERVICE					(0x40)	// bit6 Enable PathA
													// ch[0] Channel 0
#define YARM_RF_RXSERVICE					(0x40)	// bit6 Enable PathA
													// ch[0] Channel 0
													


/*! \brief The states the YARM module can reach
*/
typedef enum YARM_STATE_T {
	YARM_IDLE,
	YARM_POLLING,
	YARM_TX,
	YARM_RX,	
} YARM_State_t;

/*! \brief The command available that can sent to the 8510 module
*/
typedef enum YARM_CMD_T {
	GETVERSIONFLASH,
	SETIDLEMODE,
	READTEMPERATURE,
	GETEVENTBYTES,
} YARM_Cmd_t;

/*!
 *! \brief	Inizialize the SPI interface and the GPIO line.
 *!         Complete the power on sequence to startup the 8510 module.
 *!
 *! \param	none
 *! \return	0
*/
int32_t YARM_Init( void);

/*!
 *! \brief	This function execute the power on sequence to startup the 8510 module
 *!
 *! \param	none
 *! \return	0
*/
int32_t YARM_PowerOn( void);

/*!
 *! \brief	Power off the 8510 module
 *!
 *! \param	none
 *! \return	0
*/
int32_t YARM_PowerOff( void);

/*!
 *! \brief	Return the init state of 8510 module
 *!
 *! \param	none
 *! \return	0 not initialized, 1 initialized
*/
int32_t YARM_GetInitDone( void);

/*!
 *! \brief	This function send a command to the 8510 module
 *!
 *! \param	cmd				the command to the 8510
 *| \param  uint8_t*rxb		the receive buffer
 *! \return	<0 for error, see status_code_genare_t, otherwise the received byte.
*/
int32_t YARM_SendCmd( uint8_t*txb, uint8_t*rxb, uint8_t len);

int32_t YARM_GetEvent( void);
int32_t YARM_SetPollingMode(void);
int32_t YARM_SetIdleMode(void);
int32_t YARM_ReadRxFillLevel( uint8_t*d);
int32_t YARM_ReadTxFillLevel( uint8_t*d);
int32_t YARM_ReadRssiFillLevel( uint8_t*d);
uint8_t YARM_GetConfigEventByte( uint8_t*d);
int32_t YARM_GetEventBytes(uint8_t *data);
int32_t YARM_ReadRssiFifo(uint8_t data_size, uint8_t *data);
int32_t YARM_ReadRxFifo(uint8_t data_size, uint8_t *data);
int32_t YARM_WriteSram( uint8_t data_size, uint8_t addr_h, uint8_t addr_l, uint8_t *data);
int32_t YARM_ReadSram(uint8_t data_size, uint8_t addr_h, uint8_t addr_l, uint8_t *data);
int32_t YARM_WriteEeprom(uint8_t ee_addr_h, uint8_t ee_addr_l, uint8_t data);
int32_t YARM_ReadEeprom(uint8_t ee_addr_h, uint8_t ee_addr_l, uint8_t*d);
int32_t YARM_WriteTxFifo( uint8_t data_size, uint8_t *data);
int32_t YARM_WriteTxPreamble(uint8_t data_size, uint8_t *data);
int32_t YARM_SetSystemMode(uint8_t mode_config, uint8_t service_channel);
int32_t YARM_GetVersionFlash( uint8_t*d);
int32_t YARM_Reset(void);
uint32_t YARM_PrepareTxData( uint8_t*b, uint8_t*d, uint32_t l);
int32_t YARM_StartRSSIMeasurement( void);
int32_t YARM_GetRSSIMeasurement( uint8_t*d);
int32_t YARM_GetSysError( void);

void YARM_SPIDisable( void);
void YARM_SPIEnable( void);

unsigned char checksum (unsigned char *ptr, size_t sz);

extern uint8_t YARM_Events[2];
extern uint8_t YARM_Verbose;

#endif
