/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include <stdint.h>
#include <stdio.h>

#include "nrf.h"

#include "config.h"

#include "audio_codec_opus_api.h"
#include "opus.h"

#include <logging/log.h>
LOG_MODULE_DECLARE(audio, LOG_LEVEL_DBG);


#include "debug.h"


#define OPUS_MAX_FRAME_SIZE 3840

#if   (CONFIG_OPUS_MODE == CONFIG_OPUS_MODE_CELT)
# define OPUS_APPLICATION    OPUS_APPLICATION_RESTRICTED_LOWDELAY
//# define OPUS_ENCODER_SIZE   7180
# define OPUS_ENCODER_SIZE   11924
# define OPUS_MODE           "CELT"
# if (CONFIG_AUDIO_SAMPLING_FREQUENCY != 8000) && \
     (CONFIG_AUDIO_SAMPLING_FREQUENCY != 16000) && \
     (CONFIG_AUDIO_SAMPLING_FREQUENCY != 24000)
#  error "Selected sampling frequency is not supported by CELT codec."
# endif
# if (CONFIG_AUDIO_FRAME_SIZE_MS != 5) && \
     (CONFIG_AUDIO_FRAME_SIZE_MS != 10) && \
     (CONFIG_AUDIO_FRAME_SIZE_MS != 20)
#  error "Selected audio frame size is not supported by CELT codec."
# endif
#elif (CONFIG_OPUS_MODE == CONFIG_OPUS_MODE_SILK)
# define OPUS_APPLICATION    OPUS_APPLICATION_VOIP
# define OPUS_ENCODER_SIZE   10916
# define OPUS_MODE           "SILK"
# if (CONFIG_AUDIO_SAMPLING_FREQUENCY != 8000) && \
     (CONFIG_AUDIO_SAMPLING_FREQUENCY != 16000)
#  error "Selected sampling frequency is not supported by SILK codec."
# endif
# if (CONFIG_AUDIO_FRAME_SIZE_MS != 10) && \
     (CONFIG_AUDIO_FRAME_SIZE_MS != 20) && \
     (CONFIG_AUDIO_FRAME_SIZE_MS != 40) && \
     (CONFIG_AUDIO_FRAME_SIZE_MS != 60)
#  error "Selected audio frame size is not supported by SILK codec"
# endif
#else
# error "Unsupported OPUS Mode"
#endif

//#define OPUS_DECODER_SIZE 9224
#define OPUS_DECODER_SIZE 17944

__ALIGN(4) static uint8_t           m_opus_encoder[OPUS_ENCODER_SIZE];
__ALIGN(4) static uint8_t           m_opus_decoder[OPUS_DECODER_SIZE];

static OpusEncoder * const          m_opus_state = (OpusEncoder *)m_opus_encoder;
static OpusDecoder * const          m_opus_state_decoder = (OpusDecoder *)m_opus_decoder;


# define                            m_opus_complexity   CONFIG_OPUS_COMPLEXITY
# define                            m_opus_bitrate      ((CONFIG_OPUS_BITRATE != 0) ? CONFIG_OPUS_BITRATE : OPUS_AUTO)
# define                            m_opus_vbr          ((CONFIG_OPUS_BITRATE == 0) || (CONFIG_OPUS_VBR_ENABLED != 0))




void audio_codec_opus_init(void)
{
    LOG_INF("encoder %d", opus_encoder_get_size(CONFIG_CHANNELS));
    ASSERT(opus_encoder_get_size(CONFIG_CHANNELS) <= sizeof(m_opus_encoder));

    APP_ERROR_CHECK_BOOL(opus_encoder_init(m_opus_state, CONFIG_AUDIO_SAMPLING_FREQUENCY, CONFIG_CHANNELS, OPUS_APPLICATION) == OPUS_OK);
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(m_opus_state, OPUS_SET_BITRATE(m_opus_bitrate))                      == OPUS_OK);
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(m_opus_state, OPUS_SET_VBR(m_opus_vbr))                              == OPUS_OK);
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(m_opus_state, OPUS_SET_VBR_CONSTRAINT((m_opus_bitrate != OPUS_AUTO)))== OPUS_OK);
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(m_opus_state, OPUS_SET_COMPLEXITY(m_opus_complexity))                == OPUS_OK);
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(m_opus_state, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE))                    == OPUS_OK);
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(m_opus_state, OPUS_SET_LSB_DEPTH(16))                                == OPUS_OK);
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(m_opus_state, OPUS_SET_DTX(0))                                       == OPUS_OK);
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(m_opus_state, OPUS_SET_INBAND_FEC(0))                                == OPUS_OK); //No fec in celt mode
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(m_opus_state, OPUS_SET_PACKET_LOSS_PERC(CONFIG_LOSS_PERC))                          == OPUS_OK);//Paquet loos perc is used to determine parameter on encoder
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(m_opus_state, OPUS_SET_BANDWIDTH(OPUS_BANDWIDTH_WIDEBAND))                          == OPUS_OK);
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(m_opus_state, OPUS_SET_FORCE_CHANNELS(1))                          == OPUS_OK);

#if (CONFIG_AUDIO_FRAME_SIZE_MS == 20)
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(m_opus_state, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_20_MS))                          == OPUS_OK);
#elif (CONFIG_AUDIO_FRAME_SIZE_MS == 10)
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(m_opus_state, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_10_MS))                          == OPUS_OK);
#elif (CONFIG_AUDIO_FRAME_SIZE_MS == 5)
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(m_opus_state, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_5_MS))                          == OPUS_OK);
#else
# error "Unsupported OPUS FRAME_DURATION"
#endif


#if CONFIG_CLI_ENABLED
    m_opus_encoder_initialized = true;
#endif

    LOG_INF("decoder %d", opus_decoder_get_size(CONFIG_CHANNELS));
    ASSERT(opus_decoder_get_size(CONFIG_CHANNELS) <= sizeof(m_opus_decoder));

    APP_ERROR_CHECK_BOOL(opus_decoder_init(m_opus_state_decoder, CONFIG_AUDIO_SAMPLING_FREQUENCY, CONFIG_CHANNELS) == OPUS_OK);
}

void drv_audio_codec_encode(int16_t *input_samples, m_audio_frame_t *p_frame)
{
    p_frame->data_size = opus_encode(m_opus_state,
                             input_samples,
                             CONFIG_AUDIO_FRAME_SIZE_SAMPLES,
                             p_frame->data,
                             sizeof(p_frame->data));

    APP_ERROR_CHECK_BOOL((p_frame->data_size >= 0) && (p_frame->data_size <= OPUS_AUDIO_FRAME_SIZE_BYTES));
}

void drv_audio_codec_decode(m_audio_frame_t *p_frame, int16_t *output_samples)
{
	int ret;

   ret = opus_decode(m_opus_state_decoder,
    		(p_frame->data_size == 0)? NULL :p_frame->data,
			p_frame->data_size,
			output_samples,
			CONFIG_AUDIO_FRAME_SIZE_SAMPLES,
			0);
   if(ret != CONFIG_AUDIO_FRAME_SIZE_SAMPLES){
	   LOG_ERR("decode fail");
   }

}
/*****************************************************************************************************************/
#if 0

#include <shell/shell.h>

static int cmd_demo_ping(const struct shell *shell, size_t argc, char **argv)
{
        ARG_UNUSED(argc);
        ARG_UNUSED(argv);

        shell_print(shell, "pong");
        return 0;
}


static int cmd_opus_encoder_ctl(const struct shell *shell, size_t argc,
                           char **argv)
{
        int32_t cmd;
        int32_t param;
        int ret;

//        shell_print(shell, "argc = %d", argc);
        if(argc != 3){
            shell_print(shell, "bad number of arg. Souhld have 2 arg instead of %d", argc);
            return 0;
        }

        ret = sscanf(argv[1], "%lu", &cmd);
        if(ret != 1){
            shell_print(shell, "bad arg %s", argv[0]);
        	return 0;
        }
        ret = sscanf(argv[2], "%lu", &param);
        if(ret != 1){
            shell_print(shell, "bad arg %s", argv[1]);
        	return 0;
        }

        if(opus_encoder_ctl(m_opus_state, cmd, param) == OPUS_OK){
            shell_print(shell, "OPUS_OK");
        }else {
            shell_print(shell, "OPUS_ERROR");
        }

        return 0;
}


/* Creating subcommands (level 1 command) array for command "demo". */
SHELL_CREATE_STATIC_SUBCMD_SET(sub_demo)
{
        SHELL_CMD(opus, NULL, "cmd_opus_encoder_ctl.", cmd_opus_encoder_ctl),
        SHELL_CMD(ping,   NULL, "Ping command.", cmd_demo_ping),
        SHELL_SUBCMD_SET_END /* Array terminated. */
};

/* Creating root (level 0) command "demo" without a handler */
SHELL_CMD_REGISTER(demo, &sub_demo, "Demo commands", NULL);
#endif

