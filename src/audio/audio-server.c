/* echo.c - Networking echo server */

/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(audio, LOG_LEVEL_DBG);

#include <zephyr.h>
#include <stdio.h>
#include <sys/types.h>

#include <linker/sections.h>
#include <errno.h>


#include "audio_server.h"
#include "audio_codec_opus_api.h"
#include "audio_codec_I2S_api.h"

#define OPUS_PRIORITY 10

#define INPUT_DATA_SIZE CONFIG_AUDIO_FRAME_SIZE_SAMPLES*CONFIG_CHANNELS*sizeof(int16_t)
#define OUTPUT_DATA_SIZE CONFIG_AUDIO_FRAME_SIZE_SAMPLES*CONFIG_CHANNELS*sizeof(int16_t)

#define OPUS_ENCODE_STACK_SIZE 20000
#define OPUS_DECODE_STACK_SIZE 50000

K_THREAD_STACK_DEFINE(Audio_Rx_stack_area, OPUS_ENCODE_STACK_SIZE);
struct k_thread Audio_Rx_thread_data;
K_THREAD_STACK_DEFINE(Audio_Tx_stack_area, OPUS_DECODE_STACK_SIZE);
struct k_thread Audio_Tx_thread_data;

k_tid_t Audio_Rx_tid;
k_tid_t Audio_Tx_tid;

audio_stats_counter_t audio_stats_counter;

static u8_t Data_In[INPUT_DATA_SIZE];
static u8_t Data_Out[OUTPUT_DATA_SIZE];


static m_audio_frame_t frame_OPUS_encode;
static m_audio_frame_t frame_OPUS_decode = {.data_size = 0};


K_MSGQ_DEFINE(msgq_OPUS_OUT , sizeof(m_audio_frame_t), 1, 4);
struct k_msgq *p_msgq_OPUS_OUT = &msgq_OPUS_OUT ;
K_MSGQ_DEFINE(msgq_OPUS_IN , sizeof(m_audio_frame_t), 3, 4);
struct k_msgq *p_msgq_OPUS_IN = &msgq_OPUS_IN ;


K_MSGQ_DEFINE(msgq_Rx_IN, INPUT_DATA_SIZE/I2S_BUFFER_RATIO, I2S_BUFFER_RATIO, 4);
struct k_msgq *p_msgq_Rx_IN = &msgq_Rx_IN;
K_MSGQ_DEFINE(msgq_Tx_OUT, OUTPUT_DATA_SIZE/I2S_BUFFER_RATIO, I2S_BUFFER_RATIO*2, 4);
struct k_msgq *p_msgq_Tx_OUT = &msgq_Tx_OUT;


void audio_stats_print(){

	LOG_INF("------ Audio Stats -------");
	LOG_INF("I2S_Discared : %lu", audio_stats_counter.I2S_Discared);
	LOG_INF("Enc          : %lu", audio_stats_counter.Opus_enc);
	LOG_INF("--------------------------");
	LOG_INF("Dec       : %lu", audio_stats_counter.Opus_dec);
	LOG_INF("Gen       : %lu", audio_stats_counter.Opus_gen);
	LOG_INF("--------------------------");

	//Reset Stat
	memset(&audio_stats_counter, 0, sizeof(audio_stats_counter));
}


#ifndef CONFIG_AUDIO_CODEC_SGTL5000

static struct k_sem Rx_lock;
static void Rx_unlock(struct k_timer *timer_id);
K_TIMER_DEFINE(Rx_timer, Rx_unlock, NULL);

static void Rx_unlock(struct k_timer *timer_id)
{
	k_sem_give(&Rx_lock);
}
#endif

void Audio_Rx_Process()
{
	m_audio_frame_t *p_frame = (m_audio_frame_t *) &frame_OPUS_encode;

#if CONFIG_AUDIO_CODEC_SGTL5000
	for(int i=0; i<INPUT_DATA_SIZE; i+= INPUT_DATA_SIZE/I2S_BUFFER_RATIO){
		k_msgq_get(p_msgq_Rx_IN, &Data_In[i], K_FOREVER);
	}
#else
	k_sem_take(&Rx_lock, K_FOREVER);
#endif

	//
	audio_stats_counter.Opus_enc++;
	/*encode pcm to opus*/
	drv_audio_codec_encode((int16_t *) Data_In, p_frame);
	k_msgq_put(p_msgq_OPUS_OUT, p_frame, K_FOREVER);
}

void Audio_Tx_Process()
{
	m_audio_frame_t *p_frame = &frame_OPUS_decode;

	if(k_msgq_get(p_msgq_OPUS_IN, p_frame, K_NO_WAIT) == 0) {
		/*decode opus to pcm*/
		drv_audio_codec_decode(p_frame, (int16_t *)Data_Out);
    	audio_stats_counter.Opus_dec++;
	}else{
		//LOG_ERR("Missing frame");
		p_frame->data_size = 0;
		//
		drv_audio_codec_decode(p_frame, (int16_t *)Data_Out);
    	audio_stats_counter.Opus_gen++;
	}

	for(int i=0; i<OUTPUT_DATA_SIZE; i+= OUTPUT_DATA_SIZE/I2S_BUFFER_RATIO){
		k_msgq_put(p_msgq_Tx_OUT, &Data_Out[i], K_FOREVER);
	}
}

static void Audio_Rx_Task(){

	LOG_INF("start Audio_Rx_Task" );
#ifndef CONFIG_AUDIO_CODEC_SGTL5000
	k_sem_init(&Rx_lock, 0, UINT_MAX);
	k_timer_start(&Rx_timer, CONFIG_AUDIO_FRAME_SIZE_MS, CONFIG_AUDIO_FRAME_SIZE_MS);
#endif
	while(1){
		Audio_Rx_Process();
	}
}

static void Audio_Tx_Task(){

	LOG_INF("start Audio_Tx_Task" );
	while(1){
		Audio_Tx_Process();
	}
}


void audio_server_init()
{
	audio_codec_opus_init();

#if CONFIG_AUDIO_CODEC_SGTL5000
	audio_codec_I2S_init(p_msgq_Rx_IN, p_msgq_Tx_OUT);
#endif

	Audio_Rx_tid = k_thread_create(&Audio_Rx_thread_data, Audio_Rx_stack_area,
	                                 K_THREAD_STACK_SIZEOF(Audio_Rx_stack_area),
									 Audio_Rx_Task,
	                                 NULL, NULL, NULL,
									 OPUS_PRIORITY, 0, K_NO_WAIT);

	Audio_Tx_tid = k_thread_create(&Audio_Tx_thread_data, Audio_Tx_stack_area,
	                                 K_THREAD_STACK_SIZEOF(Audio_Tx_stack_area),
									 Audio_Tx_Task,
	                                 NULL, NULL, NULL,
									 OPUS_PRIORITY, 0, K_NO_WAIT);

	k_sleep(K_MSEC(1000));
#if CONFIG_AUDIO_CODEC_SGTL5000
	LOG_INF("Audio_start");
	audio_codec_I2S_start();
#endif
}

void audio_server_stop(){


}





