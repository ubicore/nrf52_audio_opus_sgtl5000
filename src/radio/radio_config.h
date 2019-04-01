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
#ifndef RADIO_CONFIG_H
#define RADIO_CONFIG_H


#define PACKET_BASE_ADDRESS_LENGTH  (3UL)                   //!< Packet base address length field size in bytes
//#define PACKET_STATIC_LENGTH        (0)                   //!< Packet static length in bytes
//#define PACKET_PAYLOAD_MAXSIZE      200 //(RADIO_BUF_SIZE)  //!< Packet payload maximum size in bytes

void radio_configure(uint8_t PacketSize, uint8_t power, uint32_t tx_logical_address, uint32_t rx_logical_address);

#endif
