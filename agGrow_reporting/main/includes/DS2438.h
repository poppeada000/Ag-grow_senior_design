/*
 * batteryMonitor.h
 *
 *  Created on: Apr 13, 2021
 *      Author: popad
 */

#ifndef MAIN_INCLUDES_DS2438_H_
#define MAIN_INCLUDES_DS2438_H_
#include "includes/owb.h"


void SetupAtoD(const OneWireBus * owb);
float ReadAtoD();
void oneWireMain();

#endif /* MAIN_INCLUDES_DS2438_H_ */
