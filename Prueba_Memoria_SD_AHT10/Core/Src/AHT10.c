/*
 * AHT10.c
 *
 *  Created on: Oct 14, 2022
 *      Author: User
 */

/* Debemos incluir el archivo .h de la biblioteca que estamos definiendo y la biblioteca de la HAL */
#include "AHT10.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
float tcelsius, hr; // Temperatura (en °C) y humedad relativa (%)

/* Esta función permite leer datos de temperatura y humedad del sensor AHT10. Para que el sensor
 * comience con las mediciones, es necesario enviar un comando de disparo de medición (trigger
 * measurement) junto con dos bytes de datos DATA0 y DATA1. El sensor tardará un tiempo en
 * realizar la medición y recolectar los datos (más de 75 ms). Transcurrido ese tiempo,
 * los datos de temperatura y humedad son recibidos, repartidos en 5 bytes, junto con un byte que
 * indica el estado del sensor */
void AHT10_Read(void){

	/* Definimos el comando de disparo de medición y lo enviamos a través de I2C1 */
	uint8_t triggerdata[3] = {TRIGGER_MEASUREMENT,DATA0,DATA1};
	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS_AHT10<<1, triggerdata, sizeof(triggerdata), 100);
	HAL_Delay(100); // Demora de 100 ms para que el sensor recolecte los datos

	/* Debemos recibir el dato de temperatura y humedad y guardarlo en la variable correspondiente.
	 * Para ello debemos tener en cuenta que la trama de comunicación serie recibida desde el sensor
	 * constará de 6 bytes:
	 * - BYTE 0: Byte de estado (status) que indica el estado del sensor
	 * - BYTE 1: Dato de humedad
	 * - BYTE 2: Dato de humedad
	 * - BYTE 3: Dato de humedad (4 bits más significativos) y dato de temp. (4 bits menos significativos)
	 * - BYTE 4: Dato de temperatura
	 * - BYTE 5: Dato de temperatura
	 * Tanto el dato de temperatura como el dato de humedad ocupan 20 bits.
	 */
	uint32_t temp, hum; // Variables para guardar los datos recibidos de temperatura y humedad
	uint8_t bufdata[6]; // Buffer para guardar los datos recibidos desde el sensor
	if(HAL_I2C_Master_Receive(&hi2c1, I2C_ADDRESS_AHT10<<1, bufdata, sizeof(bufdata), 100) == HAL_OK){

		/* Obtenemos el dato de humedad y lo guardamos en la variable hum */
		hum=bufdata[1]<<8; // Desplazamos el BYTE 1 8 posiciones hacia la izquierda
		hum|=bufdata[2]; // Concatenamos el BYTE 2 usando el operador lógico OR
		hum<<=4; // Desplazamos el contenido de hum 4 pos. hacia la izq.
		hum|=(bufdata[3] & 0xF0)>>4; // Concanetamos los 4 bits más significativos del BYTE 3 usando una máscara y el operador OR

		/* Obtenemos el dato de temperatura y lo guardamos en la variable temp */
		temp=(bufdata[3] & 0x0F)<<8; // Obtenemos los 4 bits menos significativos del BYTE3 usando una máscara y los desplazamos 8 pos. hacia la der.
		temp|=bufdata[4]; // Concatenamos el BYTE 4 usando el operador OR
		temp<<=8; // Desplazamos el contenido de temp 8 pos. hacia la der.
		temp|=bufdata[5]; // Concatenamos el BYTE 5 usando el operador OR

		/* Realizamos la conversión del dato de temperatura recibido desde la línea SDA a temperatura
		 * expresada en grados celsius (°C) utilizando la fórmula provista por la hoja de datos
		 * T(°C)=(temp/2^20)*200-50
		 * Como 2^20=1.048.576 --> T(°C)=(temp/1048576)*200-50 */
		tcelsius=(temp/1048576.0)*200.0-50.0;

		/* Realizamos la conversión del dato de humedad recibido desde la línea SDA a humedad
		 * relativa expresada en porcentaje, utilizando la fórmula provista por la hoja de datos
		 * HR=(hum/2^20)*100
		 * Como 2^20=1.048.576 --> HR=(hum/1048576)*100 */
		hr=(hum/1048576.0)*100.0;
	}
}
