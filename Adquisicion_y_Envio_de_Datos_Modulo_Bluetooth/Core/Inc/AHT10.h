/*
 * AHT10.h
 *
 *  Created on: Oct 14, 2022
 *      Author: User
 */

#ifndef INC_AHT10_H_
#define INC_AHT10_H_

#define I2C_ADDRESS_AHT10 0x38
#define TRIGGER_MEASUREMENT 0xAC
#define DATA0 0x33
#define DATA1 0x00

#include "globals.h"

typedef struct AHT10_DATA {
	float tcelsius; // Temperatura (en °C)
	float hr; // Humedad relativa (%)
} AHT10_DATA;

AHT10_DATA AHT10_Read(void);

#endif /* INC_AHT10_H_ */
