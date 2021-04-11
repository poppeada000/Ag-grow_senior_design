/*
 * touch_pad.h
 *
 *  Created on: Apr 6, 2021
 *      Author: popad
 */
#include <stdio.h>
#include <stdlib.h>

#ifndef MAIN_TOUCH_PAD_H_
#define MAIN_TOUCH_PAD_H_

typedef struct{
	uint16_t x;
	uint16_t y;
	uint8_t detect;
}scan_touch;

void checkTouch(scan_touch *ptr_inp);

#endif /* MAIN_TOUCH_PAD_H_ */
