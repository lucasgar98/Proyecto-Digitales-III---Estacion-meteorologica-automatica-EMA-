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
 * variables tipo bander que se activan cuando hay alguna interrupción */

extern I2C_HandleTypeDef hi2c1; // Handler asociado a la interfaz I2C1
extern I2C_HandleTypeDef hi2c2; // Handler asociado a la interfaz I2C2
extern TIM_HandleTypeDef htim1; // Variable asociada al temporizador TIM1
extern TIM_HandleTypeDef htim3; // Variable asociada al temporizador TIM3
extern UART_HandleTypeDef huart1; // Variable asociada a la USART1
extern RTC_HandleTypeDef hrtc; // Variable handler asociada al RTC
extern ADC_HandleTypeDef hadc1; // Variable asociada al ADC1
extern SPI_HandleTypeDef hspi1; // Variable handler asociada a la interfaz SPI1
extern _Bool flag_muestras; // Variable tipo bandera que se activa cada 1 ms cuando interrumpe el CH1 del TIM3 por OC
extern _Bool flag_conversion; // Variable tipo bandera que se activa cuando finaliza la conversión del ADC

/* Variables asociadas al sistema de archivos FATFS */
extern uint8_t retUSER;
extern char USERPath[4];
extern FATFS USERFatFS;
extern FIL USERFile;

#endif /* INC_GLOBALS_H_ */
