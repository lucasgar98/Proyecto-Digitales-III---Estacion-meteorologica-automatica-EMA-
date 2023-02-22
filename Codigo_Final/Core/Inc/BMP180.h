/*
 * BMP180.h
 *
 *  Created on: Oct 19, 2022
 *      Author: User
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "globals.h"

/* Definición de constantes que corresponden a las direcciones de los registros internos del BMP180 y los
 * valores a escribir en dichos registros*/
#define I2C_ADDRESS_BMP180 0x77 // Dirección I2C del sensor BMP180
#define CR_ADDRESS 0xF4 // Dirección del registro de control (CR)
#define MSB_REG_ADDRESS 0xF6 // Dirección del registro que almacena el MSB
#define LSB_REG_ADDRESS 0xF7 // Dirección del registro que almacena el LSB
#define XLSB_REG_ADDRESS 0xF8 // Dirección del registro que almacena el XLSB (modo de ultra alta resolución)
#define CR_VALUE_TEMP 0x2E // Valor a escribir en el CR para iniciar la medición de temp.
#define CR_VALUE_PRES_OSS0 0x34 // Valor a escribir en el CR para iniciar la medición de presión en modo ultra bajo consumo
#define CR_VALUE_PRES_OSS1 0x74 // Valor a escribir en el CR para iniciar la medición de presión en modo estándar
#define CR_VALUE_PRES_OSS2 0xB4 // Valor a escribir en el CR para iniciar la medición de presión en modo alta resolución
#define CR_VALUE_PRES_OSS3 0xF4 // Valor a escribir en el CR para iniciar la medición de presión en modo ultra alta resolución

/* Definimos una variable tipo enumeración para guardar los valores que puede tomar el parámetro oss
 * (oversampling setting)*/
typedef enum BMP180_OSS {
	BMP180_LOW, BMP180_STANDARD, BMP180_HIGH, BMP180_ULTRA,
} BMP180_OSS;
/* Definimos una variable tipo estructura para guardar los coeficientes de calibración*/
typedef struct BMP180_E2PROM {
	short BMP180_AC1;
	short BMP180_AC2;
	short BMP180_AC3;
	unsigned short BMP180_AC4;
	unsigned short BMP180_AC5;
	unsigned short BMP180_AC6;
	short BMP180_B1;
	short BMP180_B2;
	short BMP180_MB;
	short BMP180_MC;
	short BMP180_MD;
} BMP180_E2PROM;

/* Prototipos de funciones*/
float BMP180_Get_TrueTemperature(void);
float BMP180_Get_TruePressure(BMP180_OSS);
void BMP180_Get_AllCalCoef(void);
long BMP180_Get_RawTemperature(void);
long BMP180_Get_RawPressure(BMP180_OSS);
short BMP180_Get_CalCoef1(uint8_t,uint8_t);
unsigned short BMP180_Get_CalCoef2(uint8_t,uint8_t);
void BMP180_WriteRegister(uint8_t,uint8_t);
uint8_t BMP180_ReadRegister(uint8_t);

/* Direcciones de los bytes más significativos de los coeficientes de calibración*/
extern uint8_t DIR_MSB_COEF[11];
/* Direcciones de los bytes menos significativos de los coeficientes de calibración*/
extern uint8_t DIR_LSB_COEF[11];
extern BMP180_E2PROM calibcoef;

#endif /* INC_BMP180_H_ */
