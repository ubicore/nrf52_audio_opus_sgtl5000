/*
 * main.c
 *
 *  Created on: 12 mars 2019
 *      Author: nlantz
 */


#include <zephyr.h>
#include <stdio.h>
#include <version.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#include "nrf52_api_def.h"

#include "radio_task.h"
#include "audio_server.h"


#define STATS_PERIOD_MS 10000

static void stats_print(struct k_timer *timer_id);
K_TIMER_DEFINE(stats_timer, stats_print, NULL);


static struct k_sem quit_lock;

void quit(void)
{
	LOG_ERR("quit");
	k_sem_give(&quit_lock);
}


void panic(const char *msg)
{
	if (msg) {
		LOG_ERR("%s", msg);
	}

	for (;;) {
		k_sleep(K_FOREVER);
	}
}


static void stats_print(struct k_timer *timer_id)
{
	radio_stats_print();
	audio_stats_print();
}

void main(void)
{
	//From /otg2@otg2/zephyr/include/generated/autoconf.h
	LOG_INF("-- Zephyr   : %s ",KERNEL_VERSION_STRING);
	LOG_INF("-- Board    : %s", CONFIG_BOARD);
	LOG_INF("-- Compiled : %s %s --", __DATE__, __TIME__);

	LOG_INF("Starting...");
	k_sem_init(&quit_lock, 0, UINT_MAX);

#if 1
	//Init Radio Interface
	radio_Init();
	//Set Mode
	radio_SelectMode(MODE_NORMAL_OPERATION);
#endif

#if 1
	audio_server_init();
#endif

#if 0
	oled_test();
#endif

	//Start stats
	k_timer_start(&stats_timer, STATS_PERIOD_MS, STATS_PERIOD_MS);

	LOG_INF("wait App End...");
	k_sem_take(&quit_lock, K_FOREVER);
	LOG_INF("Stopping...");

	audio_server_stop();
}
