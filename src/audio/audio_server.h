/*
 * audio_server.h
 *
 *  Created on: 14 mars 2019
 *      Author: nlantz
 */

#ifndef SAMPLES_AUDIO_NET_OPUS_RADIO_SRC_AUDIO_AUDIO_SERVER_H_
#define SAMPLES_AUDIO_NET_OPUS_RADIO_SRC_AUDIO_AUDIO_SERVER_H_

typedef struct {
	uint32_t I2S_Discared;
	uint32_t Opus_enc;
	uint32_t Opus_dec;
	uint32_t Opus_gen;
}audio_stats_counter_t;

void audio_server_init();
void audio_server_stop();
void audio_stats_print();

#endif /* SAMPLES_AUDIO_NET_OPUS_RADIO_SRC_AUDIO_AUDIO_SERVER_H_ */
