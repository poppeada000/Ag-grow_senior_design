/*
 * cornCount.h
 *
 *  Created on: Apr 13, 2021
 *      Author: popad
 */

#ifndef MAIN_INCLUDES_CORNCOUNT_H_
#define MAIN_INCLUDES_CORNCOUNT_H_


typedef struct {
	uint16_t count;
	uint16_t countprev;
}corn;


void corn_counter(corn * ptr_data,int distance, int lim1, int lim2);
void corn_reset(corn * ptr_data);


#endif /* MAIN_INCLUDES_CORNCOUNT_H_ */
