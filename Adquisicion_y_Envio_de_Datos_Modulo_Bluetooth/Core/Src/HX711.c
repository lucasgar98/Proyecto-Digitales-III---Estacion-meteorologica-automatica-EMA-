/*
 * HX711.c
 *
 *  Created on: Oct 25, 2022
 *      Author: User
 */

#include "HX711.h"

float m=0,b=0;

void Delay_Microseconds(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

uint32_t HX711_Read_Value(_Bool format){

	uint32_t dataca2=0; // Dato leído en formato complemento a 2
	uint32_t databin=0; // Dato leído en formato binario

	  /* Iniciamos la generación de base de tiempos del TIM1 para generar las demoras en us para enviar los
	   * pulsos de reloj */
	  HAL_TIM_Base_Start(&htim1);

	/* Realizamos una encuesta (polling) sobre el pin de datos (DOUT). Mientras el pin DOUT se mantiene en 1,
	 * no hay comunicación entre el módulo HX711 y el microcontrolador, puesto que aún no ha finalizado la
	 * conversión. Cuando el pin DOUT pasa de 1 a 0, el amplificador está listo para enviar el dato */
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET){
	}
	/* Una vez que se lee un 0 en el pin DOUT, enviamos 25 pulsos de reloj a través de la señal PD_SCK para
	 * obtener el dato en complemento a 2, con una ganancia de 128 */
	for(int i=0;i<24;i++){
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
		Delay_Microseconds(1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
		Delay_Microseconds(1);
		dataca2=dataca2<<1;
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET){
		   dataca2++;
		}
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	Delay_Microseconds(1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	Delay_Microseconds(1);

	HAL_TIM_Base_Stop(&htim1);

	/* Para convertir de complemento a 2 a binario, debemos restarle 1 al complemento a 2 para obtener el
	 * complemento a 1 del número. Luego, debemos negar el complemento a 1 (es decir, convertir cada 0 en 1
	 * y cada 1 en 0) para obtener el número binario equivalente. */
	databin=16777216-dataca2;
	if(format == 1){
		return databin;
	}
	else return dataca2;
}

/* Función que promedia una determinada cantidad de muestras tomadas del HX711 cada 1 ms */
uint32_t HX711_Read_Average(uint8_t cantmuestras){

	uint32_t dataread; // Dato leído (en formato binario)
	uint64_t sumdata=0;
	uint32_t ave; // Promedio de los datos muestreados
	uint8_t cant=0;

	HAL_TIM_Base_Start_IT(&htim3);
	//HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
	while(cant<cantmuestras){
		if(flag_muestras==1){
			flag_muestras=0;
			dataread=HX711_Read_Value(1);
			sumdata=sumdata+dataread;
			cant++;
		}
	}
	HAL_TIM_Base_Stop_IT(&htim3);
	//HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
	ave=sumdata/cantmuestras;
	return ave;
}

void HX711_Set_Scale(uint8_t cantmuest){

	 uint32_t tara; // Variable para guardar el valor leído del sensor cuando no hay peso colocado
	 uint32_t val2; // Variable para guardar el valor leído del sensor cuando se coloca un peso conocido
	 //uint8_t buf[40];
	 uint8_t buf2[1];
	 //HAL_StatusTypeDef status;

	 Serial_Println("Estableciendo tara...",'A');
	 //sprintf((char*)buf,"*AEstableciendo tara...\n*");
	 //HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	 tara=HX711_Read_Average(cantmuest); // Establecemos la tara del sensor

	 Serial_Println("Inserte un peso conocido",'A');
	 //sprintf((char*)buf,"*AInserte un peso conocido\n*");
	 //HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	 //HAL_UART_Receive(&huart1, buf2, sizeof(buf2), 100)
	 while(HAL_UART_Receive(&huart1, buf2, sizeof(buf2), 1) != HAL_OK){
	 }
	 val2=HX711_Read_Average(cantmuest); // Valor asociado al peso conocido
	 m=(P2-P1)/(val2-tara);
	 b=P1-m*tara;
	 Serial_Println("Escala de peso ajustada",'A');
	 //sprintf((char*)buf,"*AEscala de peso ajustada\n*");
	 //HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	 Serial_Println("Retirar objeto",'A');
	 //sprintf((char*)buf,"*ARetirar objeto\n*");
	 //HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	 while(HAL_UART_Receive(&huart1, buf2, sizeof(buf2), 1) != HAL_OK){
	 }
}

float Leer_Pluviometro(void){

	uint32_t val;
	float peso;
	float pp;

	val=HX711_Read_Average(20);
	peso=m*val+b;
	pp=peso/(pi*pow((d/2),2)); // Precipitación (en cm)
	//pp=peso/(pi*(d/2)*(d/2));
	pp=pp*10.0; // Precipitación (en mm)

	return pp;
}
