/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
/** @file
* @addtogroup nrf_dev_radio_rx_example_main nrf_dev_radio_tx_example_main
* @{
*/

#include <stdint.h>
#include <stdbool.h>

#include "nrf.h"
#include "nrf_radio.h"
#include "radio_config.h"


/* These are set to zero as Shockburst packets don't have corresponding fields. */
#define PACKET_S1_FIELD_SIZE      (0UL)  /**< Packet S1 field size in bits. */
#define PACKET_S0_FIELD_SIZE      (0UL)  /**< Packet S0 field size in bits. */
#define RADIO_LENGTH_LENGTH_FIELD (0UL) /**< Length on air of the LENGTH field. */

/**
 * @brief Function for swapping/mirroring bits in a byte.
 * 
 *@verbatim
 * output_bit_7 = input_bit_0
 * output_bit_6 = input_bit_1
 *           :
 * output_bit_0 = input_bit_7
 *@endverbatim
 *
 * @param[in] inp is the input byte to be swapped.
 *
 * @return
 * Returns the swapped/mirrored input byte.
 */
static uint32_t swap_bits(uint32_t inp);

/**
 * @brief Function for swapping bits in a 32 bit word for each byte individually.
 * 
 * The bits are swapped as follows:
 * @verbatim
 * output[31:24] = input[24:31] 
 * output[23:16] = input[16:23]
 * output[15:8]  = input[8:15]
 * output[7:0]   = input[0:7]
 * @endverbatim
 * @param[in] input is the input word to be swapped.
 *
 * @return
 * Returns the swapped input byte.
 */
static uint32_t bytewise_bitswap(uint32_t inp);

static uint32_t swap_bits(uint32_t inp)
{
    uint32_t i;
    uint32_t retval = 0;
    
    inp = (inp & 0x000000FFUL);
    
    for (i = 0; i < 8; i++)
    {
        retval |= ((inp >> i) & 0x01) << (7 - i);     
    }
    
    return retval;    
}


static uint32_t bytewise_bitswap(uint32_t inp)
{
      return (swap_bits(inp >> 24) << 24)
           | (swap_bits(inp >> 16) << 16)
           | (swap_bits(inp >> 8) << 8)
           | (swap_bits(inp));
}

#define RADIO_LONGRANGE

void radio_configure(uint8_t PacketSize, uint8_t power, uint32_t tx_logical_address, uint32_t rx_logical_address)
{
    //Ramp-up fast : 40us
	//nrf_radio_modecnf0_set(true ,RADIO_MODECNF0_DTX_Center);


	//https://www.nordicsemi.com/DocLib/Content/Errata/nRF52840_EngB/latest/ERR/nRF52840/EngineeringB/latest/err_840

//	NRF_RADIO->POWER &= RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos;
/*
 * *(volatile uint32_t *) 0x40001740 = ((*((volatile uint32_t *) 0x40001740)) & 0x7FFF00FF) |
	0x80000000 | (((uint32_t)(196)) << 8);
	*/
	//NRF_RADIO->POWER |= RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos;


    // Reset Radio ramp-up time.
    NRF_RADIO->MODECNF0 &= (~RADIO_MODECNF0_RU_Msk);
    // Set fast ramp-up time.
    NRF_RADIO->MODECNF0 |= (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);


#ifdef RADIO_LONGRANGE
	nrf_radio_mode_set(RADIO_MODE_MODE_Ble_LR125Kbit);
#else
	nrf_radio_mode_set(RADIO_MODE_MODE_Ble_1Mbit);
#endif

	//HFXO

    // Packet configuration
    NRF_RADIO->PCNF0 = (PACKET_S1_FIELD_SIZE     << RADIO_PCNF0_S1LEN_Pos) |
                       (PACKET_S0_FIELD_SIZE     << RADIO_PCNF0_S0LEN_Pos) |
#ifdef RADIO_LONGRANGE
                       (RADIO_PCNF0_PLEN_LongRange     << RADIO_PCNF0_PLEN_Pos) |
                       (2UL << RADIO_PCNF0_CILEN_Pos) |
                       (3UL << RADIO_PCNF0_TERMLEN_Pos) |
#endif
                       (RADIO_PCNF0_CRCINC_Exclude << RADIO_PCNF0_CRCINC_Pos) |
                       (RADIO_LENGTH_LENGTH_FIELD << RADIO_PCNF0_LFLEN_Pos);

#if 1
    // Set CRC length; CRC calculation does not include the address field.
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) |
                        (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
#else
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos) |
                        (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
    // CRC Config
    if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos))
    {
        NRF_RADIO->CRCINIT = 0xFFFFUL;   // Initial value      
        NRF_RADIO->CRCPOLY = 0x11021UL;  // CRC poly: x^16+x^12^x^5+1
    }
    else if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_One << RADIO_CRCCNF_LEN_Pos))
    {
        NRF_RADIO->CRCINIT = 0xFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x107UL;  // CRC poly: x^8+x^2^x^1+1
    }
#endif

//    NRF_RADIO->CRCINIT = 0xFFFFUL;   // Initial value
//    NRF_RADIO->CRCPOLY = 0x11021UL;  // CRC poly: x^16+x^12^x^5+1

	// Packet configuration
	NRF_RADIO->PCNF1 =
#if 1
			(RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos) |
			(RADIO_PCNF1_ENDIAN_Little       << RADIO_PCNF1_ENDIAN_Pos)  |
#else
			(RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |
			(RADIO_PCNF1_ENDIAN_Big       << RADIO_PCNF1_ENDIAN_Pos)  |
#endif			
			(PACKET_BASE_ADDRESS_LENGTH   << RADIO_PCNF1_BALEN_Pos)   |
			(PacketSize         << RADIO_PCNF1_STATLEN_Pos) |
			(PacketSize       << RADIO_PCNF1_MAXLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

	//Configure Tx Power
	NRF_RADIO->TXPOWER   = (power << RADIO_TXPOWER_TXPOWER_Pos) & RADIO_TXPOWER_TXPOWER_Msk;

	//Configure shortcut for both RX and TX.
	NRF_RADIO->SHORTS = \
			RADIO_SHORTS_READY_START_Msk |
			RADIO_SHORTS_PHYEND_DISABLE_Msk |
			RADIO_SHORTS_END_DISABLE_Msk |
			RADIO_SHORTS_ADDRESS_RSSISTART_Msk |
			RADIO_SHORTS_DISABLED_RSSISTOP_Msk;

	// Radio address config
	NRF_RADIO->PREFIX0 =
	    ((uint32_t)swap_bits(0xC3) << 24) // Prefix byte of address 3 converted to nRF24L series format
	  | ((uint32_t)swap_bits(0xC2) << 16) // Prefix byte of address 2 converted to nRF24L series format
	  | ((uint32_t)swap_bits(0xC1) << 8)  // Prefix byte of address 1 converted to nRF24L series format
	  | ((uint32_t)swap_bits(0xC0) << 0); // Prefix byte of address 0 converted to nRF24L series format

	NRF_RADIO->PREFIX1 =
	    ((uint32_t)swap_bits(0xC7) << 24) // Prefix byte of address 7 converted to nRF24L series format
	  | ((uint32_t)swap_bits(0xC6) << 16) // Prefix byte of address 6 converted to nRF24L series format
	  | ((uint32_t)swap_bits(0xC4) << 0); // Prefix byte of address 4 converted to nRF24L series format

	NRF_RADIO->BASE0 = bytewise_bitswap(0x01234567UL);  // Base address for prefix 0 converted to nRF24L series format
	NRF_RADIO->BASE1 = bytewise_bitswap(0x89ABCDEFUL);  // Base address for prefix 1-7 converted to nRF24L series format

	NRF_RADIO->TXADDRESS   = tx_logical_address;  // Set device address 0 to use when transmitting
	NRF_RADIO->RXADDRESSES = (1 << rx_logical_address);  // Enable device address 0 to use to select which addresses to receive

}



/** 
 * @}
 */
