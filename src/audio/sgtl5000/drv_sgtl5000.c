/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
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

#include <zephyr.h>
#include "drv_sgtl5000.h"
#include "string.h"
#include <i2c.h>
#include <nrfx.h>
#include <math.h>

#include <logging/log.h>
LOG_MODULE_DECLARE(audio, LOG_LEVEL_DBG);

#include "debug.h"

/**
 * @brief   Teensy SGTL5000 Audio board driver for nRF52 series
 */


/* Define volume variable used to set volume of device - This is a untested feature! */
//static float                      m_volume;

static bool muted;
static uint16_t ana_ctrl;
static 	bool semi_automated;


/* Static function definitions used in this file */
static uint32_t SGTL5000_register_read(uint16_t reg_addr, uint16_t * p_reg_data);
static uint32_t SGTL5000_register_write(uint16_t reg_addr, uint16_t reg_data);
static void SGTL5000_register_write_verify(uint16_t reg_addr, uint16_t reg_data, uint16_t ro_mask);


static drv_sgtl5000_init_t * p_drv_params;

/***********************************************************/


void SGTL5000_configure(drv_sgtl5000_init_t * p_params)
{
    uint32_t                err_code;
    uint16_t                chip_id;

    p_drv_params = p_params;

    // Read ID register
    for (int i = 0; i < 5; ++i)
    {
        err_code = SGTL5000_register_read(CHIP_ID, &chip_id);
        if (err_code == NRFX_SUCCESS && (chip_id & 0xFF00) == 0xA000)
        {
            LOG_INF("Read ID OK\n");
        	break;
        }
        LOG_ERR("Read ID fail\n");
    }


	semi_automated = true;

	SGTL5000_register_write_verify(CHIP_ANA_POWER,
                                        (CHIP_ANA_POWER_REFTOP_POWERUP_Powerup << CHIP_ANA_POWER_REFTOP_POWERUP_POS)
                                        , 0xFFFF);  // VDDD is externally driven with ???V (power up reference bias currents)

    SGTL5000_register_write_verify(CHIP_REF_CTRL,
                                        (CHIP_REF_CTRL_VAG_VAL_1_575V << CHIP_REF_CTRL_VAG_VAL_POS) |
                                        (CHIP_REF_CTRL_BIAS_CTRL_p12_5 << CHIP_REF_CTRL_BIAS_CTRL_POS)
                                        , 0xFFFF);  // VAG=1.575, normal ramp, +12.5% bias current


    SGTL5000_register_write_verify(CHIP_LINE_OUT_CTRL,
                                        (CHIP_LINE_OUT_CTRL_OUT_CURRENT_0_54mA << CHIP_LINE_OUT_CTRL_OUT_CURRENT_POS) |
                                        (CHIP_LINE_OUT_CTRL_LO_VAGCNTRL_1_650V << CHIP_LINE_OUT_CTRL_LO_VAGCNTRL_POS)
                                        , 0xFFFF);  // LO_VAGCNTRL=1.65V, OUT_CURRENT=0.54mA

    SGTL5000_register_write_verify(CHIP_SHORT_CTRL,
                                        (CHIP_SHORT_CTRL_LVLADJR_125mA << CHIP_SHORT_CTRL_LVLADJR_POS) |
                                        (CHIP_SHORT_CTRL_LVLADJL_125mA << CHIP_SHORT_CTRL_LVLADJL_POS) |
                                        (CHIP_SHORT_CTRL_LVLADJC_250mA << CHIP_SHORT_CTRL_LVLADJC_POS) |
                                        (CHIP_SHORT_CTRL_MODE_LR_ShortDetectResetLatch << CHIP_SHORT_CTRL_MODE_LR_POS) |
                                        (CHIP_SHORT_CTRL_MODE_CM_ShortDetectAutoReset << CHIP_SHORT_CTRL_MODE_CM_POS)
                                        , 0xFFFF);  // allow up to 125mA

    ana_ctrl =
       (CHIP_ANA_CTRL_MUTE_LO_Mute << CHIP_ANA_CTRL_MUTE_LO_POS) |
            (CHIP_ANA_CTRL_EN_ZCD_HP_Enabled << CHIP_ANA_CTRL_EN_ZCD_HP_POS) |
            (CHIP_ANA_CTRL_MUTE_HP_Mute << CHIP_ANA_CTRL_MUTE_HP_POS) |
            (CHIP_ANA_CTRL_SELECT_ADC_LINEIN << CHIP_ANA_CTRL_SELECT_ADC_POS) |
            (CHIP_ANA_CTRL_EN_ZCD_ADC_Enabled << CHIP_ANA_CTRL_EN_ZCD_ADC_POS) |
            (CHIP_ANA_CTRL_MUTE_ADC_Mute << CHIP_ANA_CTRL_MUTE_ADC_POS);

    SGTL5000_register_write_verify(CHIP_ANA_CTRL, ana_ctrl , 0xFFFF);  // enable zero cross detectors

    muted = true;


    SGTL5000_register_write_verify(CHIP_DIG_POWER,
                                        (CHIP_DIG_POWER_ADC_POWERUP_Enable << CHIP_DIG_POWER_ADC_POWERUP_POS) |
                                        (CHIP_DIG_POWER_DAC_POWERUP_Enable << CHIP_DIG_POWER_DAC_POWERUP_POS) |
                                        (CHIP_DIG_POWER_DAP_POWERUP_Enable << CHIP_DIG_POWER_DAP_POWERUP_POS) |
                                        (CHIP_DIG_POWER_I2S_OUT_POWERUP_Enable << CHIP_DIG_POWER_I2S_OUT_POWERUP_POS) |
                                        (CHIP_DIG_POWER_I2S_IN_POWERUP_Enable << CHIP_DIG_POWER_I2S_IN_POWERUP_POS)
                                        , 0xFFFF);  // power up all digital stuff
                                        
    SGTL5000_register_write_verify(CHIP_ANA_POWER,
                                        (CHIP_ANA_POWER_DAC_MONO_Stereo << CHIP_ANA_POWER_DAC_MONO_POS) |
                                        (CHIP_ANA_POWER_PLL_POWERUP_Powerup << CHIP_ANA_POWER_PLL_POWERUP_POS) |
                                        (CHIP_ANA_POWER_VCOAMP_POWERUP_Powerup << CHIP_ANA_POWER_VCOAMP_POWERUP) |
                                        (CHIP_ANA_POWER_VAG_POWERUP_Powerup << CHIP_ANA_POWER_VAG_POWERUP_POS) |
                                        /* (CHIP_ANA_POWER_ADC_MONO_Stereo << CHIP_ANA_POWER_ADC_MONO_POS) |*/
                                        (CHIP_ANA_POWER_ADC_MONO_Mono << CHIP_ANA_POWER_ADC_MONO_POS) |
                                        (CHIP_ANA_POWER_REFTOP_POWERUP_Powerup << CHIP_ANA_POWER_REFTOP_POWERUP_POS) |
                                        (CHIP_ANA_POWER_HEADPHONE_POWERUP_Powerup << CHIP_ANA_POWER_HEADPHONE_POWERUP_POS) |
                                        (CHIP_ANA_POWER_DAC_POWERUP_Powerup << CHIP_ANA_POWER_DAC_POWERUP_POS) |
                                        (CHIP_ANA_POWER_CAPLESS_HEADPHONE_POWERUP_Powerup << CHIP_ANA_POWER_CAPLESS_HEADPHONE_POWERUP_POS) |
                                        (CHIP_ANA_POWER_ADC_POWERUP_Powerup << CHIP_ANA_POWER_ADC_POWERUP_POS) |
                                        (CHIP_ANA_POWER_LINEOUT_POWERUP_Powerup << CHIP_ANA_POWER_LINEOUT_POWERUP_POS)
                                        , 0xFFFF);  // original 0x40FF -> power up: lineout, hp, adc, dac

    SGTL5000_lineOutLevel(29);


	k_sleep(K_MSEC(1));



	//    PLL_INPUT_FREQ = SYS_MCLK

	    //Table 35. CHIP_CLK_TOP_CTRL 0x0034
	    //INPUT_FREQ_DIV2 = 0
	    //ENABLE_INT_OSC = 0
	    SGTL5000_register_write_verify(CHIP_CLK_TOP_CTRL,
	                                        (CHIP_TOP_CTRL_INPUT_FREQ_DIV2_Passthrough << CHIP_TOP_CTRL_INPUT_FREQ_DIV2_POS) |
	                                        (0 << CHIP_TOP_CTRL_ENABLE_INT_OSC_POS)
	                                        , 0xFFFF);

	    //Table 34. CHIP_PLL_CTRL 0x0032
	    //INT_DIVISOR =  FLOOR (196.608/8) => 24
	    //FRAC_DIVISOR = ((196.608/8)-24)*2048 = 1179
	    SGTL5000_register_write_verify(CHIP_PLL_CTRL,
	                                        (24 << CHIP_PLL_CTRL_INT_DIVISOR_POS) |
	                                        (1179 << CHIP_PLL_CTRL_FRAC_DIVISOR_POS)
	                                        , 0xFFFF);

		k_sleep(K_MSEC(1));


		uint8_t RATE_MODE = 0;  // Rate is 1/2 of the SYS_FS rate
		uint8_t SYS_FS =  0; //48kHz
		uint8_t	MCLK_FREQ = 0; //Use PLL

		if(p_drv_params->fs == 24000){
			RATE_MODE = 0x1;  // Rate is 1/2 of the SYS_FS rate
			SYS_FS =  0x2; //48kHz
			MCLK_FREQ = 0x3; //Use PLL
		}else if(p_drv_params->fs == 16000){
			RATE_MODE = 0x1;  // Rate is 1/2 of the SYS_FS rate
			SYS_FS =  0x0; //32kHz
			MCLK_FREQ = 0x3; //Use PLL
		}else{
			APP_ERROR_CHECK_BOOL(0);
		}
			//Table 18. CHIP_CLK_CTRL 0x0004
    SGTL5000_register_write_verify(CHIP_CLK_CTRL,
                                        (RATE_MODE << CHIP_CLK_CTRL_RATE_MODE_POS) |
                                        (SYS_FS << CHIP_CLK_CTRL_SYS_FS_POS) |
                                        (MCLK_FREQ << CHIP_CLK_CTRL_MCLK_FREQ_POS)
                                        , 0xFFFF);

    // SCLK=32*Fs, 16bit, I2S format, Master mode
    SGTL5000_register_write_verify(CHIP_I2S_CTRL,
                                        (CHIP_I2S_CTRL_SCLKFREQ_32Fs << CHIP_I2S_CTRL_SCLKFREQ_POS) |
                                        (CHIP_I2S_CTRL_MS_MASTER << CHIP_I2S_CTRL_MS_POS) |
                                        (CHIP_I2S_CTRL_DLEN_16bits << CHIP_I2S_CTRL_DLEN_POS)
                                        , 0xFFFF);

    SGTL5000_audioProcessorDisable();


    //Select SELECT_HP source : DAC
    SGTL5000_register_write_verify(CHIP_ANA_CTRL, ana_ctrl |
 	                                        (CHIP_ANA_CTRL_SELECT_HP_DAC << CHIP_ANA_CTRL_SELECT_HP_POS), 0xFFFF);

    SGTL5000_dacVolume(1.0); //0dB => 100%
    SGTL5000_dacVolumeRamp();
    //SGTL5000_dacVolumeRampDisable();

    SGTL5000_inputSelect(AUDIO_INPUT_LINEIN);
//    SGTL5000_inputSelect(AUDIO_INPUT_MIC);

    //ADC -> DAC
    //sgtl5000_register_write(CHIP_SSS_CTRL, 0x0000);

    //LINE OUT
    SGTL5000_unmuteLineout();

    //Headphone
//    SGTL5000_volumeInteger(128);
    SGTL5000_volumeInteger(100);
    // m_volume = -25.f;

    SGTL5000_unmuteHeadphone();

    SGTL5000_unmuteADC();
}


/* Reads register over TWI from audio board. */
static uint32_t SGTL5000_register_read(uint16_t reg_addr, uint16_t * p_reg_data)
{
	int ret;
	u16_t dev_addr = SGTL5000_I2C_ADDR_CS_LOW;
	uint8_t                 reg_addr_buf[2];
    uint8_t                 reg_data_buf[2];

    reg_addr_buf[0] = (reg_addr >> 8) & 0xFF;
    reg_addr_buf[1] = (reg_addr)      & 0xFF;


    memset(reg_data_buf, 0, sizeof(reg_data_buf));

	ret =  i2c_burst_read_addr(p_drv_params->i2c_dev, dev_addr, reg_addr_buf,
			2, reg_data_buf, 2);
	if (ret) {
		LOG_ERR("Fail to read from SGTL5000 @ SGTL5000_I2C_ADDR_CS_LOW\n");
		return 1;
	}

	*p_reg_data = (reg_data_buf[0] << 8 | reg_data_buf[1]);

	return NRFX_SUCCESS;
}


/* Writes register over TWI of audio board. */
static uint32_t SGTL5000_register_write(uint16_t reg_addr, uint16_t reg_data)
{
	int ret;
	u16_t dev_addr = SGTL5000_I2C_ADDR_CS_LOW;

	if (reg_addr == CHIP_ANA_CTRL) ana_ctrl = reg_data;


	uint8_t                 buf[4];
	buf[0] = (reg_addr >> 8) & 0xFF;
	buf[1] = (reg_addr)      & 0xFF;
	buf[2] = (reg_data >> 8) & 0xFF;
	buf[3] = (reg_data)      & 0xFF;

	ret = i2c_write(p_drv_params->i2c_dev, buf, 4, dev_addr);

	if (ret) {
		LOG_ERR("Fail to read from SGTL5000 @ SGTL5000_I2C_ADDR_CS_LOW\n");
		return 1;
	}

    return NRFX_SUCCESS;
}


/* Writes and verifies register over TWI of audio board. */
static void SGTL5000_register_write_verify(uint16_t reg_addr, uint16_t reg_data, uint16_t ro_mask)
{
    uint16_t read_value;
    
    do
    {
    	k_sleep(K_MSEC(1));
        SGTL5000_register_write(reg_addr, reg_data);
    	k_sleep(K_MSEC(1));
        SGTL5000_register_read(reg_addr, &read_value);
    } while ((read_value & ro_mask) != reg_data);
}



/* SGTL5000_volume_set has not been tested nor verified working */
uint32_t SGTL5000_volume_set(float volume_db)
{
    //bool    start_mclk;
    float   volume_float;
    uint8_t volume_right;
    uint8_t volume_left;
    
    int ret;

    // Valid range for analog amplifier: -51.5 to +12 dB in .5 dB steps
    if (volume_db > 12.f ||
        volume_db < -51.5f)
    {
        return NRFX_ERROR_INVALID_PARAM;
    }
    
    //m_volume = volume_db;
    
    // Value 0x00 = 12 dB (max)
    // Value 0x7F = -51.5 dB (min)
    
    volume_float = volume_db + 51.5f;
    APP_ERROR_CHECK_BOOL(volume_float >= 0.f);
    
    volume_right = (uint8_t) 0x7F - ((volume_float / 63.5f) * 127.f);
    volume_left  = volume_right;
    
    APP_ERROR_CHECK_BOOL(volume_right <= 0x7F);
    APP_ERROR_CHECK_BOOL(volume_left <= 0x7F);

    LOG_INF("Setting volume to (0x%02x) -> ", volume_right);
    
    ret = SGTL5000_register_write(CHIP_ANA_HP_CTRL, ((volume_right << 8) | volume_left));

    if(ret != NRFX_SUCCESS){
        LOG_ERR("fail\n");

    	return ret;
    }
    LOG_INF("success\n");
    
    return NRFX_SUCCESS;
}


/* SGTL5000_volume_get has not been tested nor verified working */
//uint32_t SGTL5000_volume_get(float * p_volume_db)
//{
//    if (m_state == SGTL5000_STATE_UNINITIALIZED)
//    {
//        return NRFX_ERROR_INVALID_STATE;
//    }
//    *p_volume_db = m_volume;
//
//    return NRFX_SUCCESS;
//}


/*****************************************************************************/


uint16_t SGTL5000_read(uint16_t reg)
{

	uint16_t read_value;
	SGTL5000_register_read(reg, &read_value);

	return read_value;
}


unsigned int SGTL5000_modify(uint16_t reg, uint16_t val, unsigned int iMask)
{
	uint16_t read_value;
	SGTL5000_register_read(reg, &read_value);
	uint16_t val1 = (read_value&(~iMask))|val;
	int ret;


    ret = SGTL5000_register_write(reg, val1);

    if(ret != NRFX_SUCCESS){
    	return 0;
    }

	return val1;

}
bool SGTL5000_unmuteADC(void) {
	return SGTL5000_register_write(CHIP_ANA_CTRL, ana_ctrl & ~(CHIP_ANA_CTRL_MUTE_ADC_Mute << CHIP_ANA_CTRL_MUTE_ADC_POS));
}

bool SGTL5000_muteHeadphone(void) {
	return SGTL5000_register_write(CHIP_ANA_CTRL, ana_ctrl | (1<<CHIP_ANA_CTRL_MUTE_HP_POS));
}

bool SGTL5000_unmuteHeadphone(void) {
	return SGTL5000_register_write(CHIP_ANA_CTRL, ana_ctrl & ~(1<<CHIP_ANA_CTRL_MUTE_HP_POS));
}

bool SGTL5000_muteLineout(void) {
	return SGTL5000_register_write(CHIP_ANA_CTRL, ana_ctrl | (1<<CHIP_ANA_CTRL_MUTE_LO_POS));
}

bool SGTL5000_unmuteLineout(void) {
	return SGTL5000_register_write(CHIP_ANA_CTRL, ana_ctrl & ~(1<<CHIP_ANA_CTRL_MUTE_LO_POS));
}

bool SGTL5000_inputSelect(int n) {
		if (n == AUDIO_INPUT_LINEIN) {
			return SGTL5000_register_write(CHIP_ANA_ADC_CTRL,
					 (CHIP_ANA_ADC_CTRL_ADC_VOL_M6DB_NoChange << CHIP_ANA_ADC_CTRL_ADC_VOL_M6DB_POS) |
			                                         (AUDIO_ANA_ADC_CTRL_ADC_VOL_LINEIN << CHIP_ANA_ADC_CTRL_ADC_VOL_RIGHT_POS) |
			                                         (AUDIO_ANA_ADC_CTRL_ADC_VOL_LINEIN << CHIP_ANA_ADC_CTRL_ADC_VOL_LEFT_POS)
													 ) // +7.5dB gain (1.3Vp-p full scale)
			 && SGTL5000_register_write(CHIP_ANA_CTRL, (ana_ctrl
					| (CHIP_ANA_CTRL_SELECT_ADC_LINEIN << CHIP_ANA_CTRL_SELECT_ADC_POS)
					)
					& ~(CHIP_ANA_CTRL_EN_ZCD_ADC_Enabled << CHIP_ANA_CTRL_EN_ZCD_ADC_POS)//disable ADC ZCD for LINE IN
					 ); // enable linein

//		    // LINEIN input - uncommment this section, and comment out MIC section if LINEIN input is desired
//		    // Please note that this section has not bee tested recently - might require some changes in dB settings, etc
//		    #if (AUDIO_INPUT_LINEIN == 1)
//		        sgtl5000_register_write_verify(CHIP_ANA_ADC_CTRL,
//		                                            (CHIP_ANA_ADC_CTRL_ADC_VOL_M6DB_NoChange << CHIP_ANA_ADC_CTRL_ADC_VOL_M6DB_POS) |
//		                                            (AUDIO_ANA_ADC_CTRL_ADC_VOL_LINEIN << CHIP_ANA_ADC_CTRL_ADC_VOL_RIGHT_POS) |
//		                                            (AUDIO_ANA_ADC_CTRL_ADC_VOL_LINEIN << CHIP_ANA_ADC_CTRL_ADC_VOL_LEFT_POS)
//		                                            , 0xFFFF);    // Volume control.
//		        sgtl5000_register_write_verify(CHIP_ANA_CTRL,
//		                                        (CHIP_ANA_CTRL_EN_ZCD_ADC_Enabled << CHIP_ANA_CTRL_EN_ZCD_ADC_POS)
//		                                        , 0xFFFF);
//		    #endif //(AUDIO_INPUT_LINEIN == 1)

		} else if (n == AUDIO_INPUT_MIC) {
			return SGTL5000_register_write(CHIP_MIC_CTRL, 0x0173) // mic preamp gain = +40dB
			 && SGTL5000_register_write(CHIP_ANA_ADC_CTRL, 0x088)     // input gain +12dB (is this enough?)
			 && SGTL5000_register_write(CHIP_ANA_CTRL, ana_ctrl & ~(CHIP_ANA_CTRL_SELECT_ADC_LINEIN << CHIP_ANA_CTRL_SELECT_ADC_POS)); // enable mic

//		    // MIC input - uncommment this section, and comment out LINEIN section if MIC input is desired
//		    #if (AUDIO_INPUT_MIC == 1)
//		        sgtl5000_register_write_verify(CHIP_MIC_CTRL,
//		                                            (CHIP_MIC_CTRL_GAIN_0dB << CHIP_MIC_CTRL_GAIN_POS) |
//		                                            (CHIP_MIC_CTRL_BIAS_VOLT_3_00v << CHIP_MIC_CTRL_BIAS_VOLT_POS) |
//		                                            (CHIP_MIC_CTRL_BIAS_RESISTOR_2k << CHIP_MIC_CTRL_BIAS_RESISTOR_POS)
//		                                            , 0xFFFF);
//		        sgtl5000_register_write_verify(CHIP_ANA_ADC_CTRL,
//		                                            (CHIP_ANA_ADC_CTRL_ADC_VOL_M6DB_NoChange << CHIP_ANA_ADC_CTRL_ADC_VOL_M6DB_POS) |
//		                                            (AUDIO_ANA_ADC_CTRL_ADC_VOL_MIC << CHIP_ANA_ADC_CTRL_ADC_VOL_RIGHT_POS) |
//		                                            (AUDIO_ANA_ADC_CTRL_ADC_VOL_MIC << CHIP_ANA_ADC_CTRL_ADC_VOL_LEFT_POS)
//		                                            , 0xFFFF);    // Volume control.

//		    #endif //(AUDIO_INPUT_MIC == 1)

		} else {
			return false;
		}
	}


bool SGTL5000_lineInLevel(uint8_t n) {
	return SGTL5000_lineInLevel_stereo(n, n);
}



bool SGTL5000_volumeInteger(unsigned int n)
{
	if (n == 0) {
		muted = true;
		SGTL5000_register_write(CHIP_ANA_HP_CTRL, 0x7F7F);
		return SGTL5000_muteHeadphone();
	} else if (n > 0x80) {
		n = 0;
	} else {
		n = 0x80 - n;
	}
	if (muted) {
		muted = false;
		SGTL5000_unmuteHeadphone();
	}
	n = n | (n << 8);
	return SGTL5000_register_write(CHIP_ANA_HP_CTRL, n);  // set volume
}

bool SGTL5000_volume_stereo(float left, float right)
{
	unsigned short m=((0x7F-SGTL5000_calcVol(right,0x7F))<<8)|(0x7F-SGTL5000_calcVol(left,0x7F));
	return SGTL5000_register_write(CHIP_ANA_HP_CTRL, m);
}

bool SGTL5000_micGain(unsigned int dB)
{
	unsigned int preamp_gain, input_gain;

	if (dB >= 40) {
		preamp_gain = 3;
		dB -= 40;
	} else if (dB >= 30) {
		preamp_gain = 2;
		dB -= 30;
	} else if (dB >= 20) {
		preamp_gain = 1;
		dB -= 20;
	} else {
		preamp_gain = 0;
	}
	input_gain = (dB * 2) / 3;
	if (input_gain > 15) input_gain = 15;

	return SGTL5000_register_write(CHIP_MIC_CTRL, 0x0170 | preamp_gain)
	    && SGTL5000_register_write(CHIP_ANA_ADC_CTRL, (input_gain << 4) | input_gain);
}

// CHIP_ANA_ADC_CTRL
// Actual measured full-scale peak-to-peak sine wave input for max signal
//  0: 3.12 Volts p-p
//  1: 2.63 Volts p-p
//  2: 2.22 Volts p-p
//  3: 1.87 Volts p-p
//  4: 1.58 Volts p-p
//  5: 1.33 Volts p-p
//  6: 1.11 Volts p-p
//  7: 0.94 Volts p-p
//  8: 0.79 Volts p-p
//  9: 0.67 Volts p-p
// 10: 0.56 Volts p-p
// 11: 0.48 Volts p-p
// 12: 0.40 Volts p-p
// 13: 0.34 Volts p-p
// 14: 0.29 Volts p-p
// 15: 0.24 Volts p-p
bool SGTL5000_lineInLevel_stereo(uint8_t left, uint8_t right)
{
	if (left > 15) left = 15;
	if (right > 15) right = 15;
	return SGTL5000_register_write(CHIP_ANA_ADC_CTRL, (left << 4) | right);
}

// CHIP_LINE_OUT_VOL
//  Actual measured full-scale peak-to-peak sine wave output voltage:
//  0-12: output has clipping
//  13: 3.16 Volts p-p
//  14: 2.98 Volts p-p
//  15: 2.83 Volts p-p
//  16: 2.67 Volts p-p
//  17: 2.53 Volts p-p
//  18: 2.39 Volts p-p
//  19: 2.26 Volts p-p
//  20: 2.14 Volts p-p
//  21: 2.02 Volts p-p
//  22: 1.91 Volts p-p
//  23: 1.80 Volts p-p
//  24: 1.71 Volts p-p
//  25: 1.62 Volts p-p
//  26: 1.53 Volts p-p
//  27: 1.44 Volts p-p
//  28: 1.37 Volts p-p
//  29: 1.29 Volts p-p
//  30: 1.22 Volts p-p
//  31: 1.16 Volts p-p
unsigned short SGTL5000_lineOutLevel(uint8_t n)
{
	if (n > 31) n = 31;
	else if (n < 13) n = 13;
	return SGTL5000_modify(CHIP_LINE_OUT_VOL,(n<<8)|n,(31<<8)|31);
}

unsigned short SGTL5000_lineOutLevel_stereo(uint8_t left, uint8_t right)
{
	if (left > 31) left = 31;
	else if (left < 13) left = 13;
	if (right > 31) right = 31;
	else if (right < 13) right = 13;
	return SGTL5000_modify(CHIP_LINE_OUT_VOL,(right<<8)|left,(31<<8)|31);
}

unsigned short SGTL5000_dacVolume(float n) // set both directly
{
	if ((SGTL5000_read(CHIP_ADCDAC_CTRL)&(3<<2)) != ((n>0 ? 0:3)<<2)) {
		SGTL5000_modify(CHIP_ADCDAC_CTRL,(n>0 ? 0:3)<<2,3<<2);
	}
	unsigned char m=SGTL5000_calcVol(n,0xC0);
	return SGTL5000_modify(CHIP_DAC_VOL,((0xFC-m)<<8)|(0xFC-m),65535);
}
unsigned short SGTL5000_dacVolume_stereo(float left, float right)
{
	unsigned short adcdac=((right>0 ? 0:2)|(left>0 ? 0:1))<<2;
	if ((SGTL5000_read(CHIP_ADCDAC_CTRL)&(3<<2)) != adcdac) {
		SGTL5000_modify(CHIP_ADCDAC_CTRL,adcdac,1<<2);
	}
	unsigned short m=(0xFC-SGTL5000_calcVol(right,0xC0))<<8|(0xFC-SGTL5000_calcVol(left,0xC0));
	return SGTL5000_modify(CHIP_DAC_VOL,m,65535);
}

bool SGTL5000_dacVolumeRamp()
{
	return SGTL5000_modify(CHIP_ADCDAC_CTRL, 0x300, 0x300);
}

bool SGTL5000_dacVolumeRampLinear()
{
	return SGTL5000_modify(CHIP_ADCDAC_CTRL, 0x200, 0x300);
}

bool SGTL5000_dacVolumeRampDisable()
{
	return SGTL5000_modify(CHIP_ADCDAC_CTRL, 0, 0x300);
}




unsigned short SGTL5000_adcHighPassFilterEnable(void)
{
	return SGTL5000_modify(CHIP_ADCDAC_CTRL, 0, 3);
}

unsigned short SGTL5000_adcHighPassFilterFreeze(void)
{
	return SGTL5000_modify(CHIP_ADCDAC_CTRL, 2, 3);
}

unsigned short SGTL5000_adcHighPassFilterDisable(void)
{
	return SGTL5000_modify(CHIP_ADCDAC_CTRL, 1, 3);
}


// DAP_CONTROL

unsigned short SGTL5000_audioPreProcessorEnable(void)
{
	// audio processor used to pre-process analog input before Teensy
	return SGTL5000_register_write(DAP_CONTROL, 1) && SGTL5000_register_write(CHIP_SSS_CTRL, 0x0013);
}

unsigned short SGTL5000_audioPostProcessorEnable(void)
{
	// audio processor used to post-process Teensy output before headphones/lineout
	return SGTL5000_register_write(DAP_CONTROL, 1) && SGTL5000_register_write(CHIP_SSS_CTRL, 0x0070);
}

unsigned short SGTL5000_audioProcessorDisable(void)
{
	// ADC->I2S, I2S->DAC
	return SGTL5000_register_write(CHIP_SSS_CTRL, 0x0010) && SGTL5000_register_write(DAP_CONTROL, 0);
}


// DAP_PEQ
unsigned short SGTL5000_eqFilterCount(uint8_t n) // valid to n&7, 0 thru 7 filters enabled.
{
	return SGTL5000_modify(DAP_PEQ,(n&7),7);
}

// DAP_AUDIO_EQ
unsigned short SGTL5000_eqSelect(uint8_t n) // 0=NONE, 1=PEQ (7 IIR Biquad filters), 2=TONE (tone), 3=GEQ (5 band EQ)
{
	return SGTL5000_modify(DAP_AUDIO_EQ,n&3,3);
}

unsigned short SGTL5000_eqBand(uint8_t bandNum, float n)
{
	if(semi_automated) SGTL5000_automate(1,3);
	return SGTL5000_dap_audio_eq_band(bandNum, n);
}
void SGTL5000_eqBands_mid(float bass, float mid_bass, float midrange, float mid_treble, float treble)
{
	if(semi_automated) SGTL5000_automate(1,3);
	SGTL5000_dap_audio_eq_band(0,bass);
	SGTL5000_dap_audio_eq_band(1,mid_bass);
	SGTL5000_dap_audio_eq_band(2,midrange);
	SGTL5000_dap_audio_eq_band(3,mid_treble);
	SGTL5000_dap_audio_eq_band(4,treble);
}
void SGTL5000_eqBands(float bass, float treble) // dap_audio_eq(2);
{
	if(semi_automated) SGTL5000_automate(1,2);
	SGTL5000_dap_audio_eq_band(0,bass);
	SGTL5000_dap_audio_eq_band(4,treble);
}

// SGTL5000 PEQ Coefficient loader
void SGTL5000_eqFilter(uint8_t filterNum, int *filterParameters)
{
	// TODO: add the part that selects 7 PEQ filters.
	if(semi_automated) SGTL5000_automate_opt(1,1,filterNum+1);
	SGTL5000_modify(DAP_FILTER_COEF_ACCESS,(uint16_t)filterNum,15);
	SGTL5000_register_write(DAP_COEF_WR_B0_MSB,(*filterParameters>>4)&65535);
	SGTL5000_register_write(DAP_COEF_WR_B0_LSB,(*filterParameters++)&15);
	SGTL5000_register_write(DAP_COEF_WR_B1_MSB,(*filterParameters>>4)&65535);
	SGTL5000_register_write(DAP_COEF_WR_B1_LSB,(*filterParameters++)&15);
	SGTL5000_register_write(DAP_COEF_WR_B2_MSB,(*filterParameters>>4)&65535);
	SGTL5000_register_write(DAP_COEF_WR_B2_LSB,(*filterParameters++)&15);
	SGTL5000_register_write(DAP_COEF_WR_A1_MSB,(*filterParameters>>4)&65535);
	SGTL5000_register_write(DAP_COEF_WR_A1_LSB,(*filterParameters++)&15);
	SGTL5000_register_write(DAP_COEF_WR_A2_MSB,(*filterParameters>>4)&65535);
	SGTL5000_register_write(DAP_COEF_WR_A2_LSB,(*filterParameters++)&15);
	SGTL5000_register_write(DAP_FILTER_COEF_ACCESS,(uint16_t)0x100|filterNum);
}

/* Valid values for dap_avc parameters

	maxGain; Maximum gain that can be applied
	0 - 0 dB
	1 - 6.0 dB
	2 - 12 dB

	lbiResponse; Integrator Response
	0 - 0 mS
	1 - 25 mS
	2 - 50 mS
	3 - 100 mS

	hardLimit
	0 - Hard limit disabled. AVC Compressor/Expander enabled.
	1 - Hard limit enabled. The signal is limited to the programmed threshold (signal saturates at the threshold)

	threshold
	floating point in range 0 to -96 dB

	attack
	floating point figure is dB/s rate at which gain is increased

	decay
	floating point figure is dB/s rate at which gain is reduced
*/
unsigned short SGTL5000_autoVolumeControl(uint8_t maxGain, uint8_t lbiResponse, uint8_t hardLimit, float threshold, float attack, float decay)
{
	//if(semi_automated&&(!SGTL5000_read(DAP_CONTROL)&1)) audioProcessorEnable();
	if(maxGain>2) maxGain=2;
	lbiResponse&=3;
	hardLimit&=1;
	uint8_t thresh=(pow(10,threshold/20)*0.636)*pow(2,15);
	uint8_t att=(1-pow(10,-(attack/(20*44100))))*pow(2,19);
	uint8_t dec=(1-pow(10,-(decay/(20*44100))))*pow(2,23);
	SGTL5000_register_write(DAP_AVC_THRESHOLD,thresh);
	SGTL5000_register_write(DAP_AVC_ATTACK,att);
	SGTL5000_register_write(DAP_AVC_DECAY,dec);
	return 	SGTL5000_modify(DAP_AVC_CTRL,maxGain<<12|lbiResponse<<8|hardLimit<<5,3<<12|3<<8|1<<5);
}
unsigned short SGTL5000_autoVolumeEnable(void)
{
	return SGTL5000_modify(DAP_AVC_CTRL, 1, 1);
}
unsigned short SGTL5000_autoVolumeDisable(void)
{
	return SGTL5000_modify(DAP_AVC_CTRL, 0, 1);
}

unsigned short SGTL5000_enhanceBass(float lr_lev, float bass_lev)
{
	return SGTL5000_modify(DAP_BASS_ENHANCE_CTRL,((0x3F-SGTL5000_calcVol(lr_lev,0x3F))<<8) | (0x7F-SGTL5000_calcVol(bass_lev,0x7F)), (0x3F<<8) | 0x7F);
}
unsigned short SGTL5000_enhanceBass_opt(float lr_lev, float bass_lev, uint8_t hpf_bypass, uint8_t cutoff)
{
	SGTL5000_modify(DAP_BASS_ENHANCE,(hpf_bypass&1)<<8|(cutoff&7)<<4,1<<8|7<<4);
	return SGTL5000_enhanceBass(lr_lev,bass_lev);
}
unsigned short SGTL5000_enhanceBassEnable(void)
{
	return SGTL5000_modify(DAP_BASS_ENHANCE, 1, 1);
}
unsigned short SGTL5000_enhanceBassDisable(void)
{
	return SGTL5000_modify(DAP_BASS_ENHANCE, 0, 1);
}
unsigned short SGTL5000_surroundSound(uint8_t width)
{
	return SGTL5000_modify(DAP_SGTL_SURROUND,(width&7)<<4,7<<4);
}
unsigned short SGTL5000_surroundSound_opt(uint8_t width, uint8_t select)
{
	return SGTL5000_modify(DAP_SGTL_SURROUND,((width&7)<<4)|(select&3), (7<<4)|3);
}
unsigned short SGTL5000_surroundSoundEnable(void)
{
	return SGTL5000_modify(DAP_SGTL_SURROUND, 3, 3);
}
unsigned short SGTL5000_surroundSoundDisable(void)
{
	return SGTL5000_modify(DAP_SGTL_SURROUND, 0, 3);
}

void SGTL5000_killAutomation(void) {
	semi_automated=false;
}


//n : 0.0->1.0
unsigned char SGTL5000_calcVol(float n, unsigned char range)
{
	// n=(n*(((float)range)/100))+0.499;
	n=(n*(float)range)+0.499;
	if ((unsigned char)n>range) n=range;
	return (unsigned char)n;
}

// DAP_AUDIO_EQ_BASS_BAND0 & DAP_AUDIO_EQ_BAND1 & DAP_AUDIO_EQ_BAND2 etc etc
unsigned short SGTL5000_dap_audio_eq_band(uint8_t bandNum, float n) // by signed percentage -100/+100; dap_audio_eq(3);
{
	n=(n*48)+0.499;
	if(n<-47) n=-47;
	if(n>48) n=48;
	n+=47;
	return SGTL5000_modify(DAP_AUDIO_EQ_BASS_BAND0+(bandNum*2),(unsigned int)n,127);
}

void SGTL5000_automate(uint8_t dap, uint8_t eq)
{
	//if((dap!=0)&&(!(SGTL5000_read(DAP_CONTROL)&1))) audioProcessorEnable();
	if((SGTL5000_read(DAP_AUDIO_EQ)&3) != eq) SGTL5000_eqSelect(eq);
}

void SGTL5000_automate_opt(uint8_t dap, uint8_t eq, uint8_t filterCount)
{
	SGTL5000_automate(dap,eq);
	if (filterCount > (SGTL5000_read(DAP_PEQ)&7)) SGTL5000_eqFilterCount(filterCount);
}


// if(SGTL5000_PEQ) quantization_unit=524288; if(AudioFilterBiquad) quantization_unit=2147483648;
void calcBiquad(uint8_t filtertype, float fC, float dB_Gain, float Q, uint32_t quantization_unit, uint32_t fS, int *coef)
{

// I used resources like http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt
// to make this routine, I tested most of the filter types and they worked. Such filters have limits and
// before calling this routine with varying values the end user should check that those values are limited
// to valid results.

  float A;
  if(filtertype<FILTER_PARAEQ) A=pow(10,dB_Gain/20); else A=pow(10,dB_Gain/40);
  float W0 = 2*3.14159265358979323846*fC/fS;
  float cosw=cosf(W0);
  float sinw=sinf(W0);
  //float alpha = sinw*sinh((log(2)/2)*BW*W0/sinw);
  //float beta = sqrt(2*A);
  float alpha = sinw / (2 * Q);
  float beta = sqrtf(A)/Q;
  float b0,b1,b2,a0,a1,a2;

  switch(filtertype) {
  case FILTER_LOPASS:
    b0 = (1.0F - cosw) * 0.5F; // =(1-COS($H$2))/2
    b1 = 1.0F - cosw;
    b2 = (1.0F - cosw) * 0.5F;
    a0 = 1.0F + alpha;
    a1 = 2.0F * cosw;
    a2 = alpha - 1.0F;
  break;
  case FILTER_HIPASS:
    b0 = (1.0F + cosw) * 0.5F;
    b1 = -(cosw + 1.0F);
    b2 = (1.0F + cosw) * 0.5F;
    a0 = 1.0F + alpha;
    a1 = 2.0F * cosw;
    a2 = alpha - 1.0F;
  break;
  case FILTER_BANDPASS:
    b0 = alpha;
    b1 = 0.0F;
    b2 = -alpha;
    a0 = 1.0F + alpha;
    a1 = 2.0F * cosw;
    a2 = alpha - 1.0F;
   break;
  case FILTER_NOTCH:
    b0=1;
    b1=-2*cosw;
    b2=1;
    a0=1+alpha;
    a1=2*cosw;
    a2=-(1-alpha);
  break;
  case FILTER_PARAEQ:
    b0 = 1 + (alpha*A);
    b1 =-2 * cosw;
    b2 = 1 - (alpha*A);
    a0 = 1 + (alpha/A);
    a1 = 2 * cosw;
    a2 =-(1-(alpha/A));
  break;
  case FILTER_LOSHELF:
    b0 = A * ((A+1.0F) - ((A-1.0F)*cosw) + (beta*sinw));
    b1 = 2.0F * A * ((A-1.0F) - ((A+1.0F)*cosw));
    b2 = A * ((A+1.0F) - ((A-1.0F)*cosw) - (beta*sinw));
    a0 = (A+1.0F) + ((A-1.0F)*cosw) + (beta*sinw);
    a1 = 2.0F * ((A-1.0F) + ((A+1.0F)*cosw));
    a2 = -((A+1.0F) + ((A-1.0F)*cosw) - (beta*sinw));
  break;
  case FILTER_HISHELF:
    b0 = A * ((A+1.0F) + ((A-1.0F)*cosw) + (beta*sinw));
    b1 = -2.0F * A * ((A-1.0F) + ((A+1.0F)*cosw));
    b2 = A * ((A+1.0F) + ((A-1.0F)*cosw) - (beta*sinw));
    a0 = (A+1.0F) - ((A-1.0F)*cosw) + (beta*sinw);
    a1 = -2.0F * ((A-1.0F) - ((A+1.0F)*cosw));
    a2 = -((A+1.0F) - ((A-1.0F)*cosw) - (beta*sinw));
  default:
    b0 = 0.5;
    b1 = 0.0;
    b2 = 0.0;
    a0 = 1.0;
    a1 = 0.0;
    a2 = 0.0;
  }

  a0=(a0*2)/(float)quantization_unit; // once here instead of five times there...
  b0/=a0;
  *coef++=(int)(b0+0.499);
  b1/=a0;
  *coef++=(int)(b1+0.499);
  b2/=a0;
  *coef++=(int)(b2+0.499);
  a1/=a0;
  *coef++=(int)(a1+0.499);
  a2/=a0;
  *coef++=(int)(a2+0.499);
}


