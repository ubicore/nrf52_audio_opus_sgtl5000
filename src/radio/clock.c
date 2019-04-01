/*
 * clock.c
 *
 *  Created on: 1 avr. 2019
 *      Author: nlantz
 */


#include <errno.h>
#include <zephyr/types.h>
#include <device.h>
#include <clock_control.h>
#include <drivers/clock_control/nrf_clock_control.h>


#define DEBUG_RADIO_XTAL(flag)



static struct {
	struct device *clk_hf;
} clk;



int clk_init(void)
{
	struct device *clk_k32;
	//int err;

	/* Initialize LF CLK */
	clk_k32 = device_get_binding(DT_NORDIC_NRF_CLOCK_0_LABEL "_32K");
	if (!clk_k32) {
		return -ENODEV;
	}

	clock_control_on(clk_k32, (void *)CLOCK_CONTROL_NRF_K32SRC);

	/* Initialize HF CLK */
	clk.clk_hf =
		device_get_binding(DT_NORDIC_NRF_CLOCK_0_LABEL "_16M");
	if (!clk.clk_hf) {
		return -ENODEV;
	}

	return 0;
}




int clk_on(void)
{
	int err;

	/* turn on radio clock in non-blocking mode. */
	err = clock_control_on(clk.clk_hf, NULL);
	if (!err || err == -EINPROGRESS) {
		DEBUG_RADIO_XTAL(1);
	}

	return err;
}

int clk_on_wait(void)
{
	int err;

	/* turn on radio clock in blocking mode. */
	err = clock_control_on(clk.clk_hf, (void *)1);
	if (!err || err == -EINPROGRESS) {
		DEBUG_RADIO_XTAL(1);
	}

	return err;
}

int clk_off(void)
{
	int err;

	/* turn off radio clock in non-blocking mode. */
	err = clock_control_off(clk.clk_hf, NULL);
	if (!err) {
		DEBUG_RADIO_XTAL(0);
	} else if (err == -EBUSY) {
		DEBUG_RADIO_XTAL(1);
	}

	return err;
}


