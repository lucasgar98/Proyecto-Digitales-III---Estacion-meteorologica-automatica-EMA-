/*
 * AHT10.h
 *
 *  Created on: Oct 14, 2022
 *      Author: User
 */

#ifndef INC_AHT10_H_
#define INC_AHT10_H_

#include "stm32f1xx_hal.h"

#define I2C_ADDRESS_AHT10 0x38
#define TRIGGER_MEASUREMENT 0xAC
#define DATA0 0x33
#define DATA1 0x00

void AHT10_Read(void);

extern float tcelsius, hr; // Temperatura (en °C) y humedad relativa (%)
extern I2C_HandleTypeDef hi2c1;

#endif /* INC_AHT10_H_ */
