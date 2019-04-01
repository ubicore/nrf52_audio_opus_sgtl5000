/*
 * audio_codec_I2S_api.h
 *
 *  Created on: 14 mars 2019
 *      Author: nlantz
 */

#ifndef SAMPLES_AUDIO_NET_OPUS_RADIO_SRC_AUDIO_AUDIO_CODEC_I2S_API_H_
#define SAMPLES_AUDIO_NET_OPUS_RADIO_SRC_AUDIO_AUDIO_CODEC_I2S_API_H_



/* SGTL500 driver application event types */
typedef enum
{
    DRV_SGTL5000_EVT_I2S_TX_BUF_REQ,        /* Request for I2S TX buffer */
    DRV_SGTL5000_EVT_I2S_RX_BUF_RECEIVED,   /* I2S RX buffer received */
} drv_sgtl5000_evt_type_t;

typedef struct
{
    uint32_t * p_data_to_send;          /* Pointer to buffer that should be filled  */
    uint16_t   number_of_words;         /* Buffer size in number of Words (32 bits) */
} tx_buf_req_t;

typedef struct
{
	const uint32_t * p_data_received; /* Pointer to buffer that should be filled  */
    uint16_t   number_of_words;         /* Buffer size in number of Words (32 bits) */
} rx_buf_received_t;

/* SGTL5000 driver application event */
typedef struct
{
    drv_sgtl5000_evt_type_t evt;
    union
    {
    	tx_buf_req_t tx_buf_req;
    	rx_buf_received_t rx_buf_received;
    } param;
} drv_sgtl5000_evt_t;

/* SGTL5000 driver application event handler definition */
typedef bool (* drv_sgtl5000_handler_t)(drv_sgtl5000_evt_t * p_evt);



void audio_codec_I2S_init(struct k_msgq *p_msgq_Rx, struct k_msgq *p_msgq_Tx);


uint32_t audio_codec_I2S_start(void);
uint32_t audio_codec_I2S_start_loopback(void);
uint32_t audio_codec_I2S_stop(void);


#endif /* SAMPLES_AUDIO_NET_OPUS_RADIO_SRC_AUDIO_AUDIO_CODEC_I2S_API_H_ */
