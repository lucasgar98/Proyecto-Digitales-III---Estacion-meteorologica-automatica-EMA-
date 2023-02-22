/*
 * AHT10.h
 *
 *  Created on: Oct 14, 2022
 *      Author: User
 */

#ifndef INC_AHT10_H_
#define INC_AHT10_H_

/* Definición de constantes */
#define I2C_ADDRESS_AHT10 0x38 // Dirección I2C del AHT10
#define TRIGGER_MEASUREMENT 0xAC // Comando de disparo de medición
#define DATA0 0x33 // Palabra que debe ser enviada luego del comando de disparo de medición
#define DATA1 0x00 // Palabra que debe ser enviada luego de DATA0

/* Incluimos el archivo globals.h, el cual incluye la biblioteca de la HAL y tiene definido el handler de I2C1 */
#include "globals.h"

/* Variable tipo estructura para almacenar el dato de temperatura y el dato de humeda leído del sensor */
typedef struct AHT10_DATA {
	float tcelsius; // Temperatura (en °C)
	float hr; // Humedad relativa (%)
} AHT10_DATA;

AHT10_DATA AHT10_Read(void);

#endif /* INC_AHT10_H_ */
