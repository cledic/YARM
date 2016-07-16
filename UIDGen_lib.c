#include "aes.h"
#include "UIDGen_lib.h"

//
#define AES_BUF_SIZE 4
static uint32_t AES_output_data[AES_BUF_SIZE];
/** Cipher 128 bits key array. */
const uint32_t key128[4] = {
	0xC1A0C1A0,
	0xC1A0C1A0,
	0xC1A0C1A0,
	0xC1A0C1A0
};
/* AES configuration */
struct aes_config g_aes_cfg;
/* AES instance*/
struct aes_module aes_instance;

/* \brief Receive the 128 bit long UID and return a 32bit version.
 * \param[in]  long_uid the pointer to the 128bit UID
 * \param[out] short_uid the pointer to the 32bit short UID
 * \return     0 for OK
*/
uint32_t UID_Gen( uint32_t*long_uid, uint32_t*short_uid)
{
	aes_get_config_defaults(&g_aes_cfg);
	aes_init(&aes_instance,AES, &g_aes_cfg);
	aes_enable(&aes_instance);

	/* Configure the AES. */
	g_aes_cfg.encrypt_mode = AES_ENCRYPTION;
	g_aes_cfg.key_size = AES_KEY_SIZE_128;
	g_aes_cfg.start_mode = AES_AUTO_START;
	g_aes_cfg.opmode = AES_ECB_MODE;
	g_aes_cfg.cfb_size = AES_CFB_SIZE_128;
	g_aes_cfg.lod = false;
	aes_set_config( &aes_instance,AES, &g_aes_cfg);

	/* Set the cryptographic key. */
	aes_write_key( &aes_instance, key128);

	/* The initialization vector is not used by the ECB cipher mode. */

	aes_set_new_message( &aes_instance);
	/* Write the data to be ciphered to the input data registers. */
	aes_write_input_data( &aes_instance, long_uid);
	aes_clear_new_message( &aes_instance);
	/* Wait for the end of the encryption process. */
	while (!(aes_get_status( &aes_instance) & AES_ENCRYPTION_COMPLETE)) {
	}
	aes_read_output_data( &aes_instance, AES_output_data);
    
	*short_uid=AES_output_data[2];
	
	return 0;
}
