/*
 * spi.c
 *
 *  Created on: 11 févr. 2019
 *      Author: nlantz
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(oled, LOG_LEVEL_DBG);


#include <errno.h>
#include <zephyr.h>
#include <device.h>
#include <spi.h>
#include "oled_spi.h"

#include "nrf_gpio.h"



struct spi_cs_control cs;

struct device *spi;
struct spi_config spi_cfg;




#define SPI_OLED_SELECT_CMD	SPI_PIN_CLEAR(SPI_PIN_DATA)
#define SPI_OLED_SELECT_DATA SPI_PIN_SET(SPI_PIN_DATA)

/******************************************************************************
 *
 * Description:
 *   Send a command to the SSD1351 controller
 *
 * Params:
 *   [in] cmd - the command
 *
 *****************************************************************************/
void bsp_ssd1351Command(uint8_t cmd)
{

	u8_t *mosi = &cmd;
	u16_t size =1;

	const struct spi_buf buf_tx = {
		.buf = mosi,
		.len = size
	};
	const struct spi_buf_set tx = {
		.buffers = &buf_tx,
		.count = 1
	};


	SPI_OLED_SELECT_CMD

	if (spi_transceive(spi, &spi_cfg, &tx, NULL)) {
		LOG_ERR("spi_transceive fail");
		//return BUS_FAIL;
		return;
	}
}

/******************************************************************************
 *
 * Description:
 *   Send data the SSD1351 controller
 *
 * Params:
 *   [in] data - data to send
 *
 *****************************************************************************/
void bsp_ssd1351Data(uint8_t data)
{
	u8_t *mosi = &data;
	u16_t size =1;

	const struct spi_buf buf_tx = {
		.buf = mosi,
		.len = size
	};
	const struct spi_buf_set tx = {
		.buffers = &buf_tx,
		.count = 1
	};


	SPI_OLED_SELECT_DATA

	if (spi_transceive(spi, &spi_cfg, &tx, NULL)) {
		LOG_ERR("spi_transceive fail");
		//return BUS_FAIL;
		return;
	}
}

void oled_spi_init()
{

	SPI_PIN_CFG_OUTPUT(SPI_PIN_DATA)
	SPI_OLED_SELECT_CMD


	spi = device_get_binding(DT_SPI_1_NAME);
	if (!spi) {
		LOG_ERR("Could not find SPI driver");
		return;
	}

//
	cs.gpio_dev = device_get_binding(DT_GPIO_P1_DEV_NAME);
	if (!cs.gpio_dev) {
		LOG_ERR("error");
		return;
	}
	cs.gpio_pin =12;
	cs.delay = 0;
	spi_cfg.cs = &cs;

	spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB|  SPI_WORD_SET(8) |SPI_MODE_CPOL | SPI_MODE_CPHA;
	spi_cfg.frequency = 16000000;
}



