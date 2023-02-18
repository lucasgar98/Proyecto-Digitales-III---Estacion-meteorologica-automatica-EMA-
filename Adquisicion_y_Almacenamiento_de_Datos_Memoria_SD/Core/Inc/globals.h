/*
 * globals.h
 *
 *  Created on: Nov 1, 2022
 *      Author: User
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

#include "stm32f1xx_hal.h"
#include "fatfs.h"

/* Definimos las variables globales que luego serán utilizadas por las distintas bibliotecas como variables
 * externas. Dentro de estas variables se incluyen las variables tipo handler definidas automáticamente por
 * el software al generar el código, las cuales permiten controlar los diferentes periféricos, y también las
 * variables tipo bandera que se activan cuando hay alguna interrupción */

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern RTC_HandleTypeDef hrtc;
extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi1;
extern _Bool flag_muestras;
extern _Bool flag_conversion;

//Variables asociadas al sistema de archivos FATFS
extern uint8_t retUSER;
extern char USERPath[4];
extern FATFS USERFatFS;
extern FIL USERFile;

#endif /* INC_GLOBALS_H_ */
