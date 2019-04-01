/*
 * radio_frequency.c
 *
 *  Created on: 15 juin 2016
 *      Author: nlantz
 */

#include <zephyr.h>
#include <entropy.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>




void radio_frequency_Init(uint8_t * FH_Index_Table, int size)
{
	int i;
	uint8_t temp;
   // uint32_t err_code;
    uint8_t ShuffleIndex;

	struct device *dev;

	printf("Entropy Example! %s\n", CONFIG_ARCH);

	dev = device_get_binding(CONFIG_ENTROPY_NAME);
	if (!dev) {
		printf("error: no entropy device\n");
		return;
	}

	printf("entropy device is %p, name is %s\n",
	       dev, dev->config->name);

	//Init Index Table
	for(i=0; i < size; i++)
	{
		FH_Index_Table[i] = i;
	}


	//Shuffle/Random Mix the Index Table
	for(i=0; i < size; i++)
	{
		temp = FH_Index_Table[i];
		//Get Shuffle Index in [0-79] range
		do
		{
			int r;
		    do{
				r = entropy_get_entropy(dev, &ShuffleIndex, sizeof(ShuffleIndex));
			}while(r);

			ShuffleIndex /= 3;
		}while(ShuffleIndex >= 80);
		FH_Index_Table[i] = FH_Index_Table[ShuffleIndex];
		FH_Index_Table[ShuffleIndex] = temp;
	}
}















