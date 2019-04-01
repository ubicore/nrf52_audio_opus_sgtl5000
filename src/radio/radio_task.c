/*
 * task_radio.c
 *
 *  Created on: 15 févr. 2016
 *      Author: nlantz
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <zephyr.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(radio, LOG_LEVEL_DBG);


#include "nrf52_api_def.h"
#include "nrf_gpio.h"

#include "radio_config.h"
#include "radio_task.h"
#include "nrf.h"
#include "nrf_radio.h"

#include "config.h"
#include <misc/__assert.h>

#include "radio_frequency.h"
#include "clock.h"

#define ASSERT(expr)          __ASSERT_NO_MSG(expr)

#define NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY 1

//#define MAXFREQNUMBER 79 //2400MHz + 79*1MHz
//#define RADIOPERIODLOCKED (1000*8)

#define SLOT_OCCUPENCY 4/5
//efine SLOT_OCCUPENCY 4/8 //=> more slot avalaible for LBT, but more time in Rx mode with upper consumption


#define RADIOPERIODLOCKED_US CONFIG_AUDIO_FRAME_SIZE_MS*1000*SLOT_OCCUPENCY
#define RADIOPERIODUNLOCKED_US (3*RADIOPERIODLOCKED_US)

#define RADIOSTAT
#define RADIODEBUG


#ifdef BOARD_PCA10056
#define RADIODEBUG
#endif

/********************************************************************************************/
#ifdef RADIODEBUG
#define DBG_PIN_TX	NRF_GPIO_PIN_MAP(0, 03)
#define DBG_PIN_RX	NRF_GPIO_PIN_MAP(0, 04)
#define DBG_PIN_FH_TIMER NRF_GPIO_PIN_MAP(0, 28)
#define DBG_PIN_FH_LOCKED NRF_GPIO_PIN_MAP(0, 29)
#define DBG_PIN_FH_FIRSTLOCKPERIOD NRF_GPIO_PIN_MAP(0, 30)

#define DEBUG_PIN_CFG_OUTPUT(pin)	nrf_gpio_cfg_output(pin);
#define DEBUG_PIN_SET(pin) nrf_gpio_pin_set(pin);
#define DEBUG_PIN_CLEAR(pin) nrf_gpio_pin_clear(pin);
#define DEBUG_PIN_TOGGLE(pin) nrf_gpio_pin_toggle(pin);


#else //RADIODEBUG
#define DEBUG_PIN_CFG_OUTPUT(pin)
#define DEBUG_PIN_SET(pin)
#define DEBUG_PIN_CLEAR(pin)
#define DEBUG_PIN_TOGGLE(pin)
#define PRINTANDRESET(name, value)
#endif//RADIODEBUG

/********************************************************************************************/

#define TX_LOGICAL_ADDRESS 0
#define RX_LOGICAL_ADDRESS 0

#define SIZEOFCHANNELNB 1
#define SIZEOFSTATS 4
/********************************************************************************************/

#include <nrfx/hal/nrf_timer.h>

/* Use the timer instance ID, not NRF_TIMERx directly, so that it can be checked
 * in radio_nrf5_ppi.h by the preprocessor.
 */
#define EVENT_TIMER_ID 0
#define EVENT_TIMER    _CONCAT(NRF_TIMER, EVENT_TIMER_ID)
#define TIMER_FREQUENCY NRF_TIMER_FREQ_1MHz
#define TIMER_COMPARE_CHANNEL NRF_TIMER_CC_CHANNEL0

static  NRF_TIMER_Type * p_timer = EVENT_TIMER;

#define T_RXEN_DEFAULT_US 140
#define T_RXEN_FAST_US 40

#define FH_SEQUENCE_PERIOD 80
#define MAX_DATA_SIZE 100

/**@brief States of the Radio Frequency hopping algorithm. */
typedef enum
{
	FH_UNLOCKED,               /**< Is Not Locked. */
	FH_SLAVE_FIRSTLOCKPERIOD,
	FH_LOCKED,                 /**< Is Locked. */
} radio_fh_timing_mode_t;

typedef struct
{
	uint32_t FrameNumber;
	uint8_t buf[MAX_DATA_SIZE];      /**< Pointer to the buffer. */
} __attribute__((packed)) app_radio_buffers_t;

struct FH_Data_Struct_t {
	uint8_t FH_Index_Table[FH_SEQUENCE_PERIOD];
	uint32_t GroupAddress;
}__attribute__((packed));


typedef struct {
	void (*Start)();
	void (*Process)();
	void (*Stop)();
}ModeOfOperation_t;


typedef struct {
	uint32_t Tx;
	uint32_t Tx_LBT_skip;
	uint32_t Rx;
	uint32_t Rx_LostOrBadCRC;
	uint32_t Rx_BadCRC;
	uint32_t Rx_Discared;
}stats_counter_t;

/********************************************************************************************/

static void radio_start();
static void radio_stop();
/********************************************************************************************/


struct jammer_radio_buffers_t
{
//	uint32_t FrameNumber;
	uint8_t buf[160];      /**< Pointer to the buffer. */
}__attribute__((packed));


static struct jammer_radio_buffers_t jammer_packet ={
	//	.FrameNumber = 0,
		.buf = {0xAA},
};

static stats_counter_t stats_counter;

#ifdef RADIOSTAT
//#define PRINTANDRESET(name, value) 		printf("%s : %lu => %1.6f %%\n", name, value, (float) ((float)value/((float)Stats_number_of_frame/100.0)) );
static volatile  uint32_t Stats_CRC_error = 0;
#endif //RADIOSTAT

struct radio_parameters_t radio_parameters = {
		.FH_period_locked_us = RADIOPERIODLOCKED_US,
		.FH_period_unlocked_us = RADIOPERIODUNLOCKED_US,
		//.FH_period_first_locked_us = RADIOPERIODLOCKED_US/3,
		.Lock_timeout_nb_of_period = 10,// when there is more than 10 period without resync, unlock timer and wait resync
		.DBG_radio_period = 3000, //In Tick
		//.DBG_radio_period = 1000, //In Tick
		.Tx_Power_Level = NRF_RADIO_TXPOWER_0DBM,
		//.Tx_Power_Level = RADIO_TXPOWER_TXPOWER_Neg4dBm,
		//.Tx_Power_Level = RADIO_TXPOWER_TXPOWER_Neg8dBm,
		//.Tx_Power_Level = RADIO_TXPOWER_TXPOWER_Neg12dBm,
		//.Tx_Power_Level = RADIO_TXPOWER_TXPOWER_Neg20dBm,
		//.Tx_Power_Level = NRF_RADIO_TXPOWER_NEG8DBM,
		//

		.Jammer_Power_Level = RADIO_TXPOWER_TXPOWER_Neg4dBm,
		.Pairing_Power_Level = RADIO_TXPOWER_TXPOWER_Neg8dBm,
		.Pairing_RSSI_Limit = -60, //-60dBm
		.LBT_RSSI_Limit = 0, //-70dBm
};

static bool SwitchToTx = false;
static int8_t Rssi_Level = 0;

static app_radio_buffers_t Radio_current_tx;
static app_radio_buffers_t Radio_current_rx;

static app_radio_buffers_t * m_Radio_current_tx = &Radio_current_tx;   /**< Current Radio tx buffer. */
static app_radio_buffers_t * m_Radio_current_rx = &Radio_current_rx;   /**< Current Radio rx buffer. */

extern struct k_msgq *p_msgq_OPUS_OUT;
extern struct k_msgq *p_msgq_OPUS_IN;
struct k_msgq * p_msgq_Tx;
struct k_msgq * p_msgq_Rx;



static radio_fh_timing_mode_t fh_timing_state = FH_UNLOCKED;
static uint8_t FrequencyIndex = 0;

//static void vTask_radio (void *pvParameter);

radio_mode_t RadioMode = MODE_IDLE;

struct FH_Data_Struct_t FH_Data_Struct;


/*************************************************************************/
/*************************************************************************/

void radio_stats_print(){

	LOG_INF("------ Radio Stats -------");
	LOG_INF("Tx         : %lu", stats_counter.Tx);
	LOG_INF("Tx_LBT_skip: %lu", stats_counter.Tx_LBT_skip);
	LOG_INF("--------------------------");
	LOG_INF("Rx         : %lu", stats_counter.Rx);
	LOG_INF("Rx Lost    : %lu", stats_counter.Rx_LostOrBadCRC - stats_counter.Rx_BadCRC);
	LOG_INF("Rx BadCRC  : %lu", stats_counter.Rx_BadCRC);
	LOG_INF("Rx_Discared: %lu", stats_counter.Rx_Discared);
	LOG_INF("--------------------------");

	//Reset Stat
	memset(&stats_counter, 0, sizeof(stats_counter));
}


//Frequency hopping
__STATIC_INLINE void Freq_SelectNextForPairing()
{
	//rotate index from 0->79
	FrequencyIndex += 19;
	FrequencyIndex %= 79;
}

//Frequency hopping
__STATIC_INLINE void Freq_SelectNext()
{
#if 0
	//TODO : debug : use frequence fixe
	FrequencyIndex = 20;
#endif
#if 1
	//TODO : debug use rotate index from 0->79
	FrequencyIndex += 19;
	FrequencyIndex %= 79;
#endif
	return;

	static int i = 0;
	//
	FrequencyIndex = FH_Data_Struct.FH_Index_Table[i++];
	i %= sizeof(FH_Data_Struct.FH_Index_Table);
}


static int timerlockcounter = 0;

__STATIC_INLINE void tmr_stop_and_clear()
{
	nrf_timer_task_trigger(p_timer, NRF_TIMER_TASK_STOP);
	nrf_timer_task_trigger(p_timer, NRF_TIMER_TASK_CLEAR);
	nrf_timer_event_clear(p_timer, NRF_TIMER_EVENT_COMPARE0);
	//NVIC_ClearPendingIRQ(TIMER0_IRQn + EVENT_TIMER_ID);
}

__STATIC_INLINE void tmr_start()
{
	nrf_timer_task_trigger(p_timer, NRF_TIMER_TASK_START);
	irq_enable(TIMER0_IRQn + EVENT_TIMER_ID);
}


//
__STATIC_INLINE void tmr_PeriodUpdate(radio_fh_timing_mode_t fh_timing_newstate)
{
	s32_t period_us = 0;

	tmr_stop_and_clear();

	//Called when
	if (fh_timing_newstate == FH_SLAVE_FIRSTLOCKPERIOD) {
		DEBUG_PIN_SET(DBG_PIN_FH_FIRSTLOCKPERIOD);
		period_us = radio_parameters.FH_period_first_locked_us;
	}

	//Called when device is Master (send Tx frame) or is Slave/Receiver after one First lock period
	if (fh_timing_newstate == FH_LOCKED) {
		//Set counter
		timerlockcounter = radio_parameters.Lock_timeout_nb_of_period;
		DEBUG_PIN_SET(DBG_PIN_FH_LOCKED);
		period_us = radio_parameters.FH_period_locked_us;
	}
	//Device is not locked : period used to wait a Master FH device or frame to send on as Tx as MASTER
	if (fh_timing_newstate == FH_UNLOCKED) {
		DEBUG_PIN_CLEAR(DBG_PIN_FH_LOCKED);
		period_us = radio_parameters.FH_period_unlocked_us;
	}

	nrf_timer_cc_write(p_timer, TIMER_COMPARE_CHANNEL, nrf_timer_us_to_ticks(period_us, TIMER_FREQUENCY));
	tmr_start();

	//New state
	fh_timing_state = fh_timing_newstate;
}

//Lock after first lock period and Unlock after timeout
__STATIC_INLINE void tmr_LockStateUpdate()
{
	//In Rx mode first paquet received should change timing for FH
	if (fh_timing_state == FH_SLAVE_FIRSTLOCKPERIOD) {
		DEBUG_PIN_CLEAR(DBG_PIN_FH_FIRSTLOCKPERIOD);
		tmr_PeriodUpdate(FH_LOCKED);
	} else if (fh_timing_state == FH_LOCKED){
		//If state is Locked, decrement the counter at each period except   if a new packet is sent or received
		timerlockcounter--;
		if (timerlockcounter <= 0){
			//Counter reach 0 => Unlock state
			tmr_PeriodUpdate(FH_UNLOCKED);
		}
	}
}



void Mode_Normal_Operation_Process(){
	//Lock after first lock period and Unlock after timeout
	tmr_LockStateUpdate();
	//Select next FH in FH_TABLE
	Freq_SelectNext();

	//Are there any frames waiting to be sent
	if (k_msgq_num_used_get(p_msgq_Tx) > 0){
		//switch Rx to get RSSI, then switch to Tx
		SwitchToTx = true;
	}

	//Set payload pointer
	nrf_radio_packetptr_set( m_Radio_current_rx);
	//Set frequency
	NRF_RADIO->FREQUENCY = FrequencyIndex;
	//Enable RADIO in RX mode :
	DEBUG_PIN_SET(DBG_PIN_RX);
	nrf_radio_task_trigger(NRF_RADIO_TASK_RXEN);
	return;
}



void Mode_Jammer_Process(){

	//Is Jammer
	//
	Freq_SelectNext();
	//Switch timer clock to LOCKED state if not already in Lock state as a MASTER should be
	tmr_PeriodUpdate(FH_LOCKED);
	//Set payload pointer
	nrf_radio_packetptr_set(&jammer_packet);
	//Set frequency
	NRF_RADIO->FREQUENCY = FrequencyIndex;
	//Start TX
	DEBUG_PIN_SET(DBG_PIN_TX);
	nrf_radio_task_trigger(NRF_RADIO_TASK_TXEN);
	return;
}


void Mode_Pairing_Slave_Operation_Process(){

	//Is Pairing Slave
		//
		Freq_SelectNextForPairing();
		//Set payload pointer
		nrf_radio_packetptr_set( &FH_Data_Struct);
		//Set frequency
		NRF_RADIO->FREQUENCY = FrequencyIndex;
		//Enable RADIO in RX mode :
		DEBUG_PIN_SET(DBG_PIN_RX);
		nrf_radio_task_trigger(NRF_RADIO_TASK_RXEN);
		return;
}

void Mode_Pairing_Master_Operation_Process(){

	//Is Pairing Master
		//
		Freq_SelectNextForPairing();
		//Switch timer clock to LOCKED state if not already in Lock state as a MASTER should be
		tmr_PeriodUpdate(FH_LOCKED);
		//Set payload pointer
		nrf_radio_packetptr_set(&FH_Data_Struct);
		//Set frequency
		NRF_RADIO->FREQUENCY = FrequencyIndex;
		//RadioState = STATE_TX;
		DEBUG_PIN_SET(DBG_PIN_TX);
		//Start TX
		nrf_radio_task_trigger(NRF_RADIO_TASK_TXEN);
		return;
}


ModeOfOperation_t radio_ModeOfOperation[] = {
		[MODE_NORMAL_OPERATION] = {
				.Start = NULL,
				.Process = Mode_Normal_Operation_Process,
				.Stop = NULL,
		},
		[MODE_JAMMER] = {
				.Start = NULL,
				.Process = Mode_Jammer_Process,
				.Stop = NULL,
		},
		[MODE_PAIRING_MASTER_SIDE] = {
				.Start = NULL,
				.Process = Mode_Pairing_Master_Operation_Process,
				.Stop = NULL,
		},
		[MODE_PAIRING_SLAVE_SIDE] = {
				.Start = NULL,
				.Process = Mode_Pairing_Slave_Operation_Process,
				.Stop = NULL,
		},
};


static void radio_disable()
{
	//Disable Rx if not already Disabled by END event and Shortcut
	if (nrf_radio_state_get() != RADIO_STATE_STATE_Disabled)
	{
		//Disable radio
		nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);

		while(nrf_radio_state_get() != RADIO_STATE_STATE_Disabled);

		nrf_radio_event_clear(NRF_RADIO_EVENT_DISABLED);
		NVIC_ClearPendingIRQ(RADIO_IRQn);
	}
}

void tmr_expiry_fn()
{
	ModeOfOperation_t * operation = &radio_ModeOfOperation[RadioMode];

	DEBUG_PIN_TOGGLE(DBG_PIN_FH_TIMER);

	radio_disable();
	//
	if(	operation->Process != NULL) operation->Process();
}

ISR_DIRECT_DECLARE(timer_nrf5_isr)
{
	nrf_timer_event_clear(p_timer, NRF_TIMER_EVENT_COMPARE0);
	tmr_expiry_fn();
	return 1;
}

/*************************************************************************/
static volatile uint32_t Stat_Table[10];
static volatile int Stat_Is_Printed = 1;
static volatile int Stat_Start_Print;


__STATIC_INLINE bool nrf_radio_event_check_then_clear(nrf_radio_event_t radio_event)
{
	bool result;

	result = nrf_radio_event_check(radio_event);
	if(result){
		nrf_radio_event_clear(radio_event);
	}
    return result;
}


/**@brief Function for ...
 */
static void radio_isr(void)
{
	bool check = false; //Check we handle all enabled Event.
	static int rx_last_Framenumber;

	NVIC_ClearPendingIRQ(RADIO_IRQn);

	if (nrf_radio_event_check_then_clear(NRF_RADIO_EVENT_RSSIEND))
	{
		check = true;

		//is RSSI event => TX start ?
		if(SwitchToTx)
		{
			SwitchToTx = false;
			DEBUG_PIN_CLEAR(DBG_PIN_RX);
			nrf_radio_task_trigger(NRF_RADIO_TASK_STOP);
			nrf_radio_task_trigger(NRF_RADIO_TASK_RSSISTOP);

			//Get RSSI level
			Rssi_Level =  -(NRF_RADIO->RSSISAMPLE);

			//Send a paquet if channel is not already used
			if(Rssi_Level < radio_parameters.LBT_RSSI_Limit)
			{
				if (k_msgq_get(p_msgq_Tx, m_Radio_current_tx->buf, K_NO_WAIT) == 0){
					m_Radio_current_tx->FrameNumber++;
					//Switch timer clock to LOCKED state if not already in Lock state as a MASTER should be
					tmr_PeriodUpdate(FH_LOCKED);
					//Set payload pointer
					nrf_radio_packetptr_set(m_Radio_current_tx);
					//Start TX
					DEBUG_PIN_SET(DBG_PIN_TX);
					nrf_radio_task_trigger(NRF_RADIO_TASK_TXEN);
					//Update stats
					stats_counter.Tx++;
				}
			}else{
				stats_counter.Tx_LBT_skip++;
			}
		}
	}

	if (nrf_radio_event_check_then_clear(NRF_RADIO_EVENT_DISABLED))
	{
		DEBUG_PIN_CLEAR(DBG_PIN_TX);
		DEBUG_PIN_CLEAR(DBG_PIN_RX);
		check = true;
	}

	//
	if (nrf_radio_event_check_then_clear(NRF_RADIO_EVENT_CRCOK)){
		DEBUG_PIN_CLEAR(DBG_PIN_RX);
		check = true;

		if ( RadioMode == MODE_PAIRING_SLAVE_SIDE){

			//Get RSSI level
			Rssi_Level = -(NRF_RADIO->RSSISAMPLE);
			// RSSI level check for pairing here
			if ((Rssi_Level != 0) && (Rssi_Level > radio_parameters.Pairing_RSSI_Limit)) {
				//TODO : Implement radio state machine
				radio_stop();
#ifdef BOARD_PCA10040
				nrf_gpio_pin_set(LED_4);
#endif
				RadioMode = MODE_IDLE;
			}
		}else if(RadioMode == MODE_NORMAL_OPERATION){
			//Update timing
			tmr_PeriodUpdate(FH_SLAVE_FIRSTLOCKPERIOD);

			//Check frame number
			int missing = m_Radio_current_rx->FrameNumber - (rx_last_Framenumber + 1);
			rx_last_Framenumber = m_Radio_current_rx->FrameNumber;
			//Update stats
			stats_counter.Rx++;
			stats_counter.Rx_LostOrBadCRC += missing;

			//Push new received buffer into FIFO
			if( k_msgq_put(p_msgq_Rx, m_Radio_current_rx->buf, K_NO_WAIT)){
				stats_counter.Rx_Discared++;
			}

			//Get RSSI level
			Rssi_Level =  -(NRF_RADIO->RSSISAMPLE);
		}
	}

	//
	if (nrf_radio_event_check_then_clear(NRF_RADIO_EVENT_CRCERROR)){
		check = true;

		stats_counter.Rx_BadCRC++;
	}

	//For debug only
	if(check != true)while(1);
}


//
void radio_Init()
{
	LOG_INF("Radio_Init");
	//PIN toggling for real time debug timing trace on GPIO
	DEBUG_PIN_CFG_OUTPUT(DBG_PIN_TX);
	DEBUG_PIN_CFG_OUTPUT(DBG_PIN_RX);
	DEBUG_PIN_CFG_OUTPUT(DBG_PIN_FH_TIMER);
	DEBUG_PIN_CFG_OUTPUT(DBG_PIN_FH_LOCKED);
	DEBUG_PIN_CFG_OUTPUT(DBG_PIN_FH_FIRSTLOCKPERIOD);
	//Default state
	DEBUG_PIN_CLEAR(DBG_PIN_TX);
	DEBUG_PIN_CLEAR(DBG_PIN_RX);
	DEBUG_PIN_CLEAR(DBG_PIN_FH_TIMER);
	DEBUG_PIN_CLEAR(DBG_PIN_FH_LOCKED);
	DEBUG_PIN_CLEAR(DBG_PIN_FH_FIRSTLOCKPERIOD);


	clk_init();
	clk_on_wait();
}

//
static void radio_configure_normal_op()
{
//	int CompleteFrameSizeInbyte
	int DurationInUS;
//	uint8_t PacketSize = sizeof(app_radio_buffers_t);

	p_msgq_Tx = p_msgq_OPUS_OUT;
	p_msgq_Rx = p_msgq_OPUS_IN;

//	PacketSize = (internal_parameters.spi_pcm_size / codec_parameters.nb_splited_channel)+ SIZEOFCHANNELNB + SIZEOFSTATS;
//	PacketSize = internal_parameters.radio_buffer_size;

	/**/
	//Get timer value
	//Frame Size in bit
	//PREAMBULE (8bit or 16bit) => 8 + PACKET_BASE_ADDRESS_LENGTH => 4*8  + PREFIX ADD  => 1*8 + PACKET_S0_FIELD_SIZE  => 0 + LFLEN => 8 + PACKET_S1_FIELD_SIZE => 0 + PAYLOAD => m_Radio_current_rx->lenght*8  + CRC (2 bit)
	//8*(PREAMBULE+1)+8*PACKET_BASE_ADDRESS_LENGTH+8*1+PACKET_S0_FIELD_SIZE+LFLEN+PACKET_S1_FIELD_SIZE+(m_Radio_current_rx->lenght*8)+RADIO_CRCCNF_LEN_Two
	//8+4*8+8+0+8+0+2=58 bit
	//CompleteFrameSizeInbyte = 58 + 8*p_msgq_Tx->msg_size;
	//#define RATE1MHz 1000000UL
	//Tx duration in us @ 1Mhz
	//DurationInUS = CompleteFrameSizeInbyte*8;
	/**/

	DurationInUS = 2005; //Tx time mesured with digital oscilloscope 2.05ms


	//if (DurationInUS != 386) while(1); //TODO : nlantz : check de la taille a faire
	radio_parameters.FH_period_first_locked_us = (radio_parameters.FH_period_locked_us - DurationInUS)/2 - T_RXEN_FAST_US;

	LOG_INF("Configure radio for msg of %d bytes ", p_msgq_Tx->msg_size);

	ASSERT( p_msgq_Tx->msg_size <= MAX_DATA_SIZE);

	// Set radio configuration parameters
	radio_configure((sizeof(Radio_current_tx.FrameNumber) + p_msgq_Tx->msg_size), radio_parameters.Tx_Power_Level, TX_LOGICAL_ADDRESS, RX_LOGICAL_ADDRESS);

}
//
static void radio_configure_for_pairing_Slave()
{
	//uint8_t PacketSize = sizeof(struct FH_Data_Struct_t);
	uint8_t PacketSize = FH_SEQUENCE_PERIOD;

	// Set radio configuration parameters
	radio_configure(PacketSize, radio_parameters.Tx_Power_Level, 0, RX_LOGICAL_ADDRESS);

}

//
static void radio_configure_jammer()
{
	uint8_t PacketSize = sizeof(app_radio_buffers_t);
	static uint32_t JammerAddress = TX_LOGICAL_ADDRESS;

//	PacketSize = (spi_slave_parameters.size_buffer_spi / codec_parameters.nb_splited_channel)+ SIZEOFCHANNELNB + SIZEOFSTATS;
	//PacketSize = internal_parameters.radio_buffer_size;

	// Set radio configuration parameters
	radio_configure(PacketSize, radio_parameters.Jammer_Power_Level, JammerAddress++, 0);
}

//
static void radio_configure_for_pairing_Master()
{
	//uint8_t PacketSize = sizeof(struct FH_Data_Struct_t);
	uint8_t PacketSize = FH_SEQUENCE_PERIOD;

	p_msgq_Rx = NULL;

	//Create a random FH sequence
	radio_frequency_Init(FH_Data_Struct.FH_Index_Table, FH_SEQUENCE_PERIOD);

	// Set radio configuration parameters
	radio_configure(PacketSize, radio_parameters.Pairing_Power_Level, TX_LOGICAL_ADDRESS, 0);
}

#define RADIO_IRQPriority    6
#define PPI_CHANNEL 1

static void radio_start()
{
	irq_disable(RADIO_IRQn);

	//Clear event
	nrf_radio_event_clear(NRF_RADIO_EVENT_DISABLED);
	nrf_radio_event_clear(NRF_RADIO_EVENT_CRCOK);
	nrf_radio_event_clear(NRF_RADIO_EVENT_CRCERROR);
	nrf_radio_event_clear(NRF_RADIO_EVENT_RXREADY);
	nrf_radio_event_clear(NRF_RADIO_EVENT_RSSIEND);


	//Create shortcut between RXREADY _EVENT and RSSI_START_Task with PPI channel
	 NRF_PPI->CH[PPI_CHANNEL].EEP = (uint32_t) &NRF_RADIO->EVENTS_RXREADY;
	 NRF_PPI->CH[PPI_CHANNEL].TEP = (uint32_t) &NRF_RADIO->TASKS_RSSISTART;
	 NRF_PPI->CHENSET = PPI_CHENSET_CH0_Enabled << (PPI_CHENSET_CH0_Pos + PPI_CHANNEL);

	//Clear all Interrupt flag
	nrf_radio_int_disable(0xFF);
	//Enable INT
	nrf_radio_int_enable(NRF_RADIO_INT_DISABLED_MASK);
	nrf_radio_int_enable(NRF_RADIO_INT_CRCOK_MASK);
	nrf_radio_int_enable(NRF_RADIO_INT_CRCERROR_MASK);
	nrf_radio_int_enable(NRF_RADIO_INT_RSSIEND_MASK);

	tmr_stop_and_clear();

	nrf_timer_mode_set(p_timer, NRF_TIMER_MODE_TIMER);
	nrf_timer_frequency_set(p_timer, TIMER_FREQUENCY);
	nrf_timer_bit_width_set(p_timer, NRF_TIMER_BIT_WIDTH_32);
	nrf_timer_shorts_enable(p_timer, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);

	nrf_timer_int_enable(p_timer, NRF_TIMER_INT_COMPARE0_MASK);

	IRQ_DIRECT_CONNECT(TIMER0_IRQn + EVENT_TIMER_ID, NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY, timer_nrf5_isr, 0);

	IRQ_DIRECT_CONNECT(RADIO_IRQn, RADIO_IRQPriority, radio_isr, 0);
	NVIC_ClearPendingIRQ(RADIO_IRQn);
	NVIC_EnableIRQ(RADIO_IRQn);

	//Init and start timer for FH
	tmr_PeriodUpdate(FH_UNLOCKED);
}

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))
//
static void radio_stop()
{
	tmr_stop_and_clear();
	nrf_timer_int_disable(p_timer, NRF_TIMER_INT_COMPARE0_MASK);
	irq_disable(TIMER0_IRQn + EVENT_TIMER_ID);

	//
	nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
	//Disable IT
	irq_disable(RADIO_IRQn);
}

#ifdef BOARD_PCA10040
	#define LED_TRACE(x) nrf_gpio_pin_clear(x);
#else
	#define LED_TRACE(x)
#endif



void radio_SelectMode(radio_mode_t NewRadioMode)
{
	//Stop radio if runing
	if (RadioMode != MODE_IDLE) radio_stop();

#ifdef BOARD_PCA10040

	//Reset All LED
	nrf_gpio_pins_set(LEDS_MASK);
#endif
	//
	RadioMode = NewRadioMode;
	//
	if (NewRadioMode == MODE_IDLE) return;

	//Configure radio
	switch (NewRadioMode)
	{
	case MODE_NORMAL_OPERATION:
		radio_configure_normal_op();
		LED_TRACE(LED_1);
		break;
	case MODE_JAMMER:
		radio_configure_jammer();
		LED_TRACE(LED_2);
		break;
	case MODE_PAIRING_MASTER_SIDE:
		radio_configure_for_pairing_Master();
		LED_TRACE(LED_3);
		break;
	case MODE_PAIRING_SLAVE_SIDE:
		radio_configure_for_pairing_Slave();
		LED_TRACE(LED_4);
		break;
	default:
		//configASSERT(0);
		break;
	}

	//Start radio
	radio_start();
}

