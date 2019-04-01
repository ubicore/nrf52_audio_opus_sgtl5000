/*
 * spi_bridge_protocol.h
 *
 *  Created on: 11 févr. 2016
 *      Author: nlantz
 */

#ifndef NRF52_API_DEF_H_
#define NRF52_API_DEF_H_

/****************************************************************/
/****************************************************************/

/**@brief radio parameter  structure definitions. */
struct radio_parameters_t
{
	uint32_t FH_period_first_locked_us;
	uint32_t FH_period_locked_us;
	uint32_t FH_period_unlocked_us;
	uint32_t Lock_timeout_nb_of_period;
	uint32_t DBG_radio_period;
	uint8_t Tx_Power_Level;
	uint8_t Pairing_Power_Level;
	uint8_t Jammer_Power_Level;
	int8_t Pairing_RSSI_Limit;
	int8_t LBT_RSSI_Limit;
}__attribute__((packed));

/**@brief radio parameter  structure definitions. */
struct internal_parameters_t
{
	uint8_t spi_pcm_size;
	uint8_t radio_buffer_size;
}__attribute__((packed));

/**@brief  Firmware parameter  structure definitions. */
struct firmware_parameters_t
{
	struct radio_parameters_t radio_parameters;
}__attribute__((packed));

/**@brief  Firmware parameter  structure definitions. */
struct firmware_parameters_interface_t
{
	struct radio_parameters_t radio_parameters;
}__attribute__((packed));

/**@brief SPI slave bridge nRF52 command definitions. */
typedef enum
{
	CMD_IDLE,
	CMD_DATA_RD,	/**< Read Data received on radio. */
	CMD_DATA_WR,	 /**< Write Data on radio. */
	CMD_INIT_BRIDGE,
	CMD_SET_RADIO_MODE,
	CMD_GET_RADIO_MODE,
	CMD_RESET,
	CMD_OTHER,
} nRF_command_t;

/**@brief Modes of the Radio. */
typedef enum
{
	MODE_IDLE,                 /**< Is in IDLE Mode. */
	MODE_NORMAL_OPERATION,     /**< Is in IDLE Mode. */
	MODE_JAMMER,               /**< Is in IDLE Mode. */
	MODE_PAIRING_MASTER_SIDE,  /**< Is in IDLE Mode. */
	MODE_PAIRING_SLAVE_SIDE,  /**< Is in IDLE Mode. */
} radio_mode_t;


#endif /* NRF52_API_DEF_H_ */
