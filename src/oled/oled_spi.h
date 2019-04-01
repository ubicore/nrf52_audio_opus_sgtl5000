/*
 * oled_spi.h
 *
 *  Created on: 11 févr. 2019
 *      Author: nlantz
 */

#ifndef SRC_OLED_OLED_SPI_H_
#define SRC_OLED_OLED_SPI_H_

void oled_spi_init();
void bsp_ssd1351Data(uint8_t data);
void bsp_ssd1351Command(uint8_t cmd);

#define SPI_PIN_CS	NRF_GPIO_PIN_MAP(1, 12) 			//CS
#define SPI_PIN_DATA	NRF_GPIO_PIN_MAP(1, 13)			//D/C

#define OLED_RESET	NRF_GPIO_PIN_MAP(1, 14) 			//

#define SPI_PIN_CFG_OUTPUT(pin)	nrf_gpio_cfg_output(pin);
#define SPI_PIN_SET(pin) nrf_gpio_pin_set(pin);
#define SPI_PIN_CLEAR(pin) nrf_gpio_pin_clear(pin);
#define SPI_PIN_TOGGLE(pin) nrf_gpio_pin_toggle(pin);


#endif /* SRC_OLED_OLED_SPI_H_ */
