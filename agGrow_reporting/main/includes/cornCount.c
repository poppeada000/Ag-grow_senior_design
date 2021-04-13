/*
 * cornCount.c
 *
 *  Created on: Apr 13, 2021
 *      Author: popad
 */
#include <stdio.h>
#include "cornCount.h"

corn corn_data;
int reset;


void corn_counter(corn* ptr_data, int dist, int lim1, int lim2)
{
	corn * ptr_temp = &corn_data;

	if(dist >= lim2)
	{
		reset = 1;
	}

	else if((reset == 1) && (dist <= lim1))
	{
		ptr_temp->count += 1;
		//printf("%d",ptr_temp->count);
		reset = 0;
	}
	printf("%d", reset);

	ptr_data->count = ptr_temp->count;
}

void corn_reset(corn * ptr_data)
{
	reset = 0;
	corn * ptr_temp = &corn_data;
	ptr_temp->count = ptr_data->count;
}

uint16_t getCount(){
	return (uint16_t) corn_data.count;
}
