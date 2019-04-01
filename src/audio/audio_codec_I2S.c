/*
 * audio_codec_task.c
 *
 *  Created on: 11 mars 2019
 *      Author: nlantz
 */

#include <logging/log.h>
LOG_MODULE_DECLARE(audio, LOG_LEVEL_DBG);

#include <stdio.h>
#include <string.h>
#include <zephyr.h>

#include "nrf_gpio.h"
#include "config.h"
#include <nrfx_i2s.h>
#include <i2c.h>
#include <nrfx/hal/nrf_timer.h>
#include "drv_sgtl5000.h"
#include "audio_codec_I2S_api.h"
#include "audio_server.h"

#include "debug.h"


#define NRF_LOG_INFO LOG_INF

#define NRFX_I2S_CONFIG_IRQ_PRIORITY 1


static struct k_msgq *p_msgq_I2S_Rx;
static struct k_msgq *p_msgq_I2S_Tx;

//#define AUDIO_FRAME_WORDS                   320
#define AUDIO_FRAME_WORDS                   CONFIG_AUDIO_FRAME_SIZE_SAMPLES/I2S_BUFFER_RATIO // Sample Left and right in one word
static uint32_t  m_i2s_rx_buffer[2][AUDIO_FRAME_WORDS];// Double buffered
static uint32_t  m_i2s_tx_buffer[2][AUDIO_FRAME_WORDS];

#ifdef DEMO
/* Include car sample to demonstrate that sample can be played from application as well */
#define SAMPLE_LEN                  67200
extern const uint8_t                car_sample[SAMPLE_LEN];
static uint8_t * p_sample           = (uint8_t *)car_sample;
static uint32_t sample_idx          = 0;
#endif


/* Define i2s handler to handle audio data to and from audio device */
static drv_sgtl5000_handler_t     m_i2s_evt_handler;
/* Define i2s configuration for audio device */
static nrfx_i2s_config_t       m_i2s_config;


typedef enum
{
	recorder,
	player,
} audio_type_t;

audio_type_t type;

extern audio_stats_counter_t audio_stats_counter;

/* Definiton of SGTL5000 driver states */
static enum
{
    SGTL5000_STATE_UNINITIALIZED,   /* Not initialized */
    SGTL5000_STATE_IDLE,            /* Initialized, but not running */
    SGTL5000_STATE_RUNNING,         /* Actively streaming audio to/from application */
    SGTL5000_STATE_RUNNING_LOOPBACK,/* Actively running audio loopback (I2S_IN -> I2S_OUT) */
} m_state = SGTL5000_STATE_UNINITIALIZED;

static bool i2s_sgtl5000_driver_evt_handler(drv_sgtl5000_evt_t * p_evt)
{
	u16_t PCM_Data_RX[AUDIO_FRAME_WORDS]; //mono
	u16_t PCM_Data_TX[AUDIO_FRAME_WORDS*2]; //stereo
	
    bool ret = true;
    //NRF_LOG_INFO("i2s_sgtl5000_driver_evt_handler %d", p_evt->evt);

    switch (p_evt->evt)
    {
        case DRV_SGTL5000_EVT_I2S_RX_BUF_RECEIVED:
            {

                if(type == recorder)
                {
                    //NRF_LOG_INFO("i2s_sgtl5000_driver_evt_handler RX BUF RECEIVED");
                    //Handle RX
                    uint16_t * p_buffer  = (uint16_t *) p_evt->param.rx_buf_received.p_data_received;
                    size_t received = p_evt->param.rx_buf_received.number_of_words; //stereo

                    if(received == AUDIO_FRAME_WORDS){
                    	//Stereo to Mono : copy left sample only
                    	for(int i=0; i< received;i++){
                    		//Copy sample
                    		PCM_Data_RX[i] = p_buffer[i*2];
                    	}

                    	if(k_msgq_put(p_msgq_I2S_Rx, (uint8_t *) PCM_Data_RX, K_NO_WAIT)){
                    		audio_stats_counter.I2S_Discared++;
                    	}
                    }

                }else {
                	//Do nothing
                }
            }
            break;
        case DRV_SGTL5000_EVT_I2S_TX_BUF_REQ:
            {
                uint16_t * p_buffer  = (uint16_t *) p_evt->param.tx_buf_req.p_data_to_send;
                uint32_t i2s_buffer_size_words = p_evt->param.tx_buf_req.number_of_words;

                if(type == player)
                {
                	if(k_msgq_get(p_msgq_I2S_Tx, PCM_Data_TX, K_NO_WAIT) == 0){
                		//Mono to Stereo
                		for(int i=0; i< (i2s_buffer_size_words);i++){
                			//Copy sample
                			p_buffer[i*2] = PCM_Data_TX[i];
                			p_buffer[i*2+1] = PCM_Data_TX[i];
                		}
                	}else{
                		LOG_ERR("Empty TX");

                        /* Clear pcm buffer */
                        memset(p_buffer, 0, i2s_buffer_size_words*sizeof(uint32_t));
                	}
                }
            }
            break;
    }

    return ret;
}


/***************************/

/* I2S pin mapping */
#define SGTL5000_PIN_SYS_MCLK       (32+1)
#define SGTL5000_I2S_PIN_BCLK       (32+2)
#define SGTL5000_I2S_PIN_LRCLK      (32+3)
#define SGTL5000_I2S_PIN_RX         (32+4)
#define SGTL5000_I2S_PIN_TX         (32+5)


/***************************/

/* Utility EGU interrupt - This interrupt is used to stop the I2S peripheral when the transmission is done */
#define SGTL5000_EGU_INSTANCE       NRF_EGU3
#define SGTL5000_EGU_IRQn           SWI3_EGU3_IRQn
#define SGTL5000_EGU_IRQHandler     SWI3_EGU3_IRQHandler
//#define DRV_SGTL5000_EGU_IRQPriority    APP_IRQ_PRIORITY_LOWEST
#define SGTL5000_EGU_IRQPriority    6
#define SGTL5000_EGU_TASK_STREAMING_STOP    0

/***************************/




#define I2C_DEV_NAME DT_I2C_0_NAME

struct device *i2c_dev;
//u32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_FAST) | I2C_MODE_MASTER;
u32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_MASTER;


static void init_i2c()
{
	i2c_dev = device_get_binding(I2C_DEV_NAME);

	if (!i2c_dev) {
		LOG_ERR("Cannot get I2C device");
		return ;
	}

	/* 1. Verify i2c_configure() */
	if (i2c_configure(i2c_dev, i2c_cfg)) {
		LOG_ERR("I2C config failed");
		return ;
	}
	   //Disable pull-up resistors on SCL and SDA (already mounted on audio board)

}





/* I2S event handler matching SDK v14.2 implementation - will be invoked from the SDK v15 I2S event handler in order to ensure compatibility */
static void i2s_data_handler_old(uint32_t const * p_data_received, uint32_t * p_data_to_send, uint16_t number_of_words)
{
    // Non-NULL value in 'p_data_received' indicates that a new portion of
    // data has been received and should be processed.
    if (p_data_received != NULL)
    {
//        if (m_state != SGTL5000_STATE_RUNNING)
//        {
//            // I2S only running in order to provide clock
//            return;
//        }
//
        drv_sgtl5000_evt_t evt;

        evt.evt                                     = DRV_SGTL5000_EVT_I2S_RX_BUF_RECEIVED;
        evt.param.rx_buf_received.number_of_words   = number_of_words;
        evt.param.rx_buf_received.p_data_received   = p_data_received;

        m_i2s_evt_handler(&evt);
    }

    // Non-NULL value in 'p_data_to_send' indicates that the driver needs
    // a new portion of data to send.
    if (p_data_to_send != NULL)
    {
//        if (m_state != SGTL5000_STATE_RUNNING)
//        {
//            // I2S only running in order to provide clock
//            return;
//        }

        drv_sgtl5000_evt_t evt;
        bool continue_running;

        // Request for I2S data to transmit
        evt.evt                              = DRV_SGTL5000_EVT_I2S_TX_BUF_REQ;
        evt.param.tx_buf_req.number_of_words = number_of_words;
        evt.param.tx_buf_req.p_data_to_send  = p_data_to_send;

        continue_running = m_i2s_evt_handler(&evt);
        if (!continue_running)
        {
        	SGTL5000_EGU_INSTANCE->TASKS_TRIGGER[SGTL5000_EGU_TASK_STREAMING_STOP] = 1;
        }
    }
}


/* I2S event handler. Based on the module state, will play sample, playback microphone data, or forward events to the application */
static void i2s_data_handler(nrfx_i2s_buffers_t const * p_released,
                         uint32_t                      status)
{
    if (!(status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED))
    {
        return;
    }

    if (p_released == NULL)
    {
        // Regardless of module state, when p_released is NULL, we provide the next buffers (to keep implementation a little simpler)
        nrfx_i2s_buffers_t const next_buffers = {
            .p_rx_buffer = m_i2s_rx_buffer[1],
            .p_tx_buffer = m_i2s_tx_buffer[1],
        };
        i2s_data_handler_old(NULL, (uint32_t *) next_buffers.p_tx_buffer, AUDIO_FRAME_WORDS);
        APP_ERROR_CHECK(nrfx_i2s_next_buffers_set(&next_buffers));
    }
    else if (p_released->p_rx_buffer == NULL)
    {
        // If RX buffer is NULL, no data has been received, and we need to provide the next buffers. Nothing else done (to keep implementation a little simpler).
        nrfx_i2s_buffers_t const next_buffers = {
                .p_rx_buffer = m_i2s_rx_buffer[1],
                .p_tx_buffer = m_i2s_tx_buffer[1],
        };
        i2s_data_handler_old(NULL, (uint32_t *) next_buffers.p_tx_buffer, AUDIO_FRAME_WORDS);
        APP_ERROR_CHECK(nrfx_i2s_next_buffers_set(&next_buffers));
    }
    else
    {
        // This is the normal standard I2S running state. We check module states here and not before to keep implementation a little simpler.
        if (m_state == SGTL5000_STATE_RUNNING)
        {
            // If we are forwarding events to the application, we let the "old" event handler take care of that
        	i2s_data_handler_old(p_released->p_rx_buffer, (uint32_t *)p_released->p_tx_buffer, AUDIO_FRAME_WORDS);
            APP_ERROR_CHECK(nrfx_i2s_next_buffers_set(p_released));
        }
//        else if (m_state == SGTL5000_STATE_RUNNING_LOOPBACK)
//        {
//            // Forward I2S_IN to I2S_OUT
//            nrfx_i2s_buffers_t const next_buffers = {
//                .p_rx_buffer = (uint32_t *)p_released->p_tx_buffer,
//                .p_tx_buffer = (uint32_t *)p_released->p_rx_buffer,
//            };
//            APP_ERROR_CHECK(nrfx_i2s_next_buffers_set(&next_buffers));
//        }
        else
        {
            // We do not handle this scenario
        }
    }
}


/* EGU interrupt handler. This handler will take care of stopping the I2S peripheral when requested. */
void SGTL5000_EGU_IRQHandler(void)
{
    if (SGTL5000_EGU_INSTANCE->EVENTS_TRIGGERED[SGTL5000_EGU_TASK_STREAMING_STOP] != 0)
    {
    	SGTL5000_EGU_INSTANCE->EVENTS_TRIGGERED[SGTL5000_EGU_TASK_STREAMING_STOP] = 0;
        nrfx_i2s_stop();
        m_state = SGTL5000_STATE_IDLE;
    }
}
/***********************************************************/

#define SYS_MCLK_TIMER_ID 1
#define SYS_MCLK_TIMER_FREQUENCY NRF_TIMER_FREQ_16MHz
#define SYS_MCLK_TIMER_COMPARE_CHANNEL NRF_TIMER_CC_CHANNEL0

static NRF_TIMER_Type * p_timer = _CONCAT(NRF_TIMER, SYS_MCLK_TIMER_ID);

#define GPIOTE_CHANNEL 0
#define PPI_CHANNEL 0

static void SGTL5000_sys_mclk_enable()
{
	nrf_gpio_cfg_output(SGTL5000_PIN_SYS_MCLK); //Configure pin 18 as output


	 //Configure GPIOTE to toggle pin 18
	 NRF_GPIOTE->CONFIG[GPIOTE_CHANNEL] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos |
	                         GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
							 SGTL5000_PIN_SYS_MCLK << GPIOTE_CONFIG_PSEL_Pos |
	                         GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos;


	 //Configure timer
	 nrf_timer_mode_set(p_timer, NRF_TIMER_MODE_TIMER);
	 nrf_timer_frequency_set(p_timer, SYS_MCLK_TIMER_FREQUENCY);
	 nrf_timer_bit_width_set(p_timer, NRF_TIMER_BIT_WIDTH_8);
	 nrf_timer_shorts_enable(p_timer, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);
	 nrf_timer_cc_write(p_timer, SYS_MCLK_TIMER_COMPARE_CHANNEL, 1);
	 //nrf_timer_int_enable(p_timer, NRF_TIMER_INT_COMPARE0_MASK);
	 //irq_enable(NRF5_IRQ_TIMER0_IRQn + SYS_MCLK_TIMER_ID);
	 nrf_timer_task_trigger(p_timer, NRF_TIMER_TASK_START);

	//Configure PPI
	 NRF_PPI->CH[PPI_CHANNEL].EEP = (uint32_t) &p_timer->EVENTS_COMPARE[SYS_MCLK_TIMER_COMPARE_CHANNEL];
	 NRF_PPI->CH[PPI_CHANNEL].TEP = (uint32_t) &NRF_GPIOTE->TASKS_OUT[GPIOTE_CHANNEL];

	 NRF_PPI->CHENSET = PPI_CHENSET_CH0_Enabled << (PPI_CHENSET_CH0_Pos + PPI_CHANNEL);

}



static drv_sgtl5000_init_t sgtl_drv_params;


void audio_codec_I2S_init(struct k_msgq *p_msgq_Rx, struct k_msgq *p_msgq_Tx)
{
    uint32_t                err_code;

	p_msgq_I2S_Rx = p_msgq_Rx;
	p_msgq_I2S_Tx = p_msgq_Tx;


    // Enable audio
#ifdef CONFIG_AUDIO_CODEC_RECORDER
	LOG_INF("recorder ==================>  radio ");
	type = recorder;
#else
	LOG_INF("radio ==================> player");
	type = player;

#endif


    NRF_LOG_INFO("size of  m_i2s_tx_buffer %d words", AUDIO_FRAME_WORDS);
    //NRF_LOG_INFO("i2s_initial_tx_buffer addr1: %d, addr2: %d", m_i2s_tx_buffer, m_i2s_tx_buffer + I2S_BUFFER_SIZE_WORDS/2);
    //NRF_LOG_INFO("i2s_initial_Rx_buffer addr1: %d, addr2: %d", m_i2s_rx_buffer, m_i2s_rx_buffer + I2S_BUFFER_SIZE_WORDS/2);


    init_i2c();

    sgtl_drv_params.fs                      = CONFIG_AUDIO_SAMPLING_FREQUENCY;
    sgtl_drv_params.i2c_dev                 = i2c_dev;

    SGTL5000_sys_mclk_enable();
    k_sleep(K_MSEC(1));
    SGTL5000_configure(&sgtl_drv_params);


	{   //SGTL5000_init_EGU
		SGTL5000_EGU_INSTANCE->INTENCLR = 0xFFFFFFFF;
		SGTL5000_EGU_INSTANCE->INTENSET = 0x0000FFFF;

		IRQ_DIRECT_CONNECT(SGTL5000_EGU_IRQn, SGTL5000_EGU_IRQPriority, SGTL5000_EGU_IRQHandler, 0);
		NVIC_ClearPendingIRQ(SGTL5000_EGU_IRQn);
		NVIC_EnableIRQ(SGTL5000_EGU_IRQn);
	}

    {    //SGTL5000_init_I2S
    	// Update configuration
    	m_i2s_evt_handler                           = i2s_sgtl5000_driver_evt_handler;

    	// Initialize I2S
    	m_i2s_config.sck_pin      = SGTL5000_I2S_PIN_BCLK;
    	m_i2s_config.lrck_pin     = SGTL5000_I2S_PIN_LRCLK;
    	m_i2s_config.mck_pin      = NRFX_I2S_PIN_NOT_USED;
    	m_i2s_config.sdout_pin    = SGTL5000_I2S_PIN_TX;
    	m_i2s_config.sdin_pin     = SGTL5000_I2S_PIN_RX;
    	m_i2s_config.irq_priority = NRFX_I2S_CONFIG_IRQ_PRIORITY;
    	m_i2s_config.mode         = NRF_I2S_MODE_SLAVE;
    	m_i2s_config.format       = NRF_I2S_FORMAT_I2S;
    	m_i2s_config.alignment    = NRF_I2S_ALIGN_LEFT;
    	m_i2s_config.sample_width = NRF_I2S_SWIDTH_16BIT;
    	m_i2s_config.channels     = NRF_I2S_CHANNELS_STEREO;
    	m_i2s_config.mck_setup    = NRF_I2S_MCK_DISABLED;
    	m_i2s_config.ratio  = 0;

    	IRQ_DIRECT_CONNECT(NRF52_IRQ_I2S_IRQn, m_i2s_config.irq_priority, nrfx_i2s_irq_handler, 0);

        err_code = nrfx_i2s_init(&m_i2s_config, i2s_data_handler);
        if (err_code != NRFX_SUCCESS)
        {
            return;
        }
    }

    m_state = SGTL5000_STATE_IDLE;

    LOG_INF("Audio initialization done.");

    //loopback test
//   SGTL5000_start_loopback();
//   k_sleep(K_MSEC(2000));
//   SGTL5000_stop();
}



/* Starts the I2S peripheral - which will communicate with the audio board and forward I2S events to the application. */
uint32_t audio_codec_I2S_start(void)
{
    if (m_state == SGTL5000_STATE_IDLE)
    {
        m_state = SGTL5000_STATE_RUNNING;
        nrfx_i2s_buffers_t const initial_buffers = {
            .p_rx_buffer =  m_i2s_rx_buffer[0],
            .p_tx_buffer =  m_i2s_tx_buffer[0],
        };

        /* Clear tx double buffer */
        memset((uint32_t * )initial_buffers.p_tx_buffer, 0, AUDIO_FRAME_WORDS*sizeof(uint32_t));

        (void)nrfx_i2s_start(&initial_buffers, AUDIO_FRAME_WORDS, 0);

        return NRFX_SUCCESS;
    }

    return NRFX_ERROR_INVALID_STATE;
}

//
///* Starts the I2S peripheral, forwards I2S_IN input to I2S_OUT. No events are forwarded to the application. */
//uint32_t audio_codec_I2S_start_loopback(void)
//{
//    if (m_state == SGTL5000_STATE_IDLE)
//    {
//        m_state = SGTL5000_STATE_RUNNING_LOOPBACK;
//        nrfx_i2s_buffers_t const initial_buffers = {
//            .p_tx_buffer = m_external_i2s_buffer.tx_buffer,
//            .p_rx_buffer = m_external_i2s_buffer.rx_buffer,
//        };
//        (void)nrfx_i2s_start(&initial_buffers, (m_external_i2s_buffer.buffer_size_words/2), 0);
//
//        return NRFX_SUCCESS;
//    }
//
//    return NRFX_ERROR_INVALID_STATE;
//}


/* Stops the I2S peripheral */
uint32_t audio_codec_I2S_stop(void)
{
    if ((m_state == SGTL5000_STATE_RUNNING ) || (m_state == SGTL5000_STATE_RUNNING_LOOPBACK ))
    {
        nrfx_i2s_stop();
        m_state = SGTL5000_STATE_IDLE;

        return NRFX_SUCCESS;
    }

    return NRFX_ERROR_INVALID_STATE;
}

