/*
 * HX711.c
 *
 *  Created on: Oct 25, 2022
 *      Author: User
 */

#include "HX711.h"

HX711_DATA rectapeso;

/* Función que genera la demora en microsegundos necesaria para enviar los pulsos de reloj al HX711. Utiliza
 * como base de tiempos el timer TIM1 */
void Delay_Microseconds(uint8_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

/* Rutina que lee el valor del HX711 enviando los pulsos de reloj correspondientes y lo convierte de
 * complemento a 2 a binario */
uint32_t HX711_Read_Value(_Bool format){

	uint32_t dataca2=0; // Dato leído en formato complemento a 2
	uint32_t databin=0; // Dato leído en formato binario

	/* Iniciamos la generación de base de tiempos del TIM1 para generar las demoras en us para enviar los
	 * pulsos de reloj */
	HAL_TIM_Base_Start(&htim1);

	/* Realizamos una encuesta (polling) sobre el pin de datos (DOUT). Mientras el pin DOUT se mantiene en 1,
	 * no hay comunicación entre el módulo HX711 y el microcontrolador, puesto que aún no ha finalizado la
	 * conversión. Cuando el pin DOUT pasa de 1 a 0, el amplificador está listo para enviar el dato */
	while(HAL_GPIO_ReadPin(DOUT_GPIO_Port, DOUT_Pin) == GPIO_PIN_SET){
	}
	/* Una vez que se lee un 0 en el pin DOUT, enviamos 25 pulsos de reloj a través de la señal PD_SCK para
	 * obtener el dato en complemento a 2, con una ganancia de 128 */
	for(int i=0;i<24;i++){
	    HAL_GPIO_WritePin(PD_SCK_GPIO_Port, PD_SCK_Pin, GPIO_PIN_SET); // Nivel alto de la señal SCK
		Delay_Microseconds(1);
		HAL_GPIO_WritePin(PD_SCK_GPIO_Port, PD_SCK_Pin, GPIO_PIN_RESET); // Nivel bajo de la señal de reloj
		Delay_Microseconds(1);
		/* Guardamos cada uno de los bits recibidos en una variable */
		dataca2=dataca2<<1;
		if(HAL_GPIO_ReadPin(DOUT_GPIO_Port, DOUT_Pin) == GPIO_PIN_SET){
		   dataca2++;
		}
	}
	HAL_GPIO_WritePin(PD_SCK_GPIO_Port, PD_SCK_Pin, GPIO_PIN_SET);
	Delay_Microseconds(1);
	HAL_GPIO_WritePin(PD_SCK_GPIO_Port, PD_SCK_Pin, GPIO_PIN_RESET);
	Delay_Microseconds(1);

	/* Finalizamos la generación de base de tiempos del TIM1 */
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
	uint64_t sumdata=0; // Variable para sumar los valores leídos
	uint32_t ave; // Promedio de los datos muestreados
	uint8_t cant=0; // Variable contador para contar las muestras tomadas


	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1); // Iniciamos el TIM3 en modo comparación de salida
	/* Cada 1 ms, leemos un valor del HX711 */
	while(cant<cantmuestras){
		if(flag_muestras==1){
			flag_muestras=0; // Reseteamos la bandera
			dataread=HX711_Read_Value(1); // Leemos el dato
			sumdata=sumdata+dataread; // Vamos sumando los valores leídos
			cant++; // Incrementamos el contador
		}
	}
	HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1); // Detenemos el modo output compare del TIM3
	ave=sumdata/cantmuestras; // Calculamos el promedio de las muestras
	return ave;
}

/* Rutina que realiza el ajuste de la escala de peso a partir de un peso conocido y de la tara (valor leído
 * cuando no hay ningún peso colocado). Para ello envía un mensaje al usuario pidiéndole que inserte un
 * peso conocido */
void HX711_Set_Scale(uint8_t cantmuest){

	 uint32_t tara; // Variable para guardar el valor leído del sensor cuando no hay peso colocado
	 uint32_t val2; // Variable para guardar el valor leído del sensor cuando se coloca un peso conocido
	 uint8_t buf2; // Variable para guardar el caracter recibido

	 /* Solicitamos al usuario que inserte un peso conocido. El programa se queda esperando a que el usuario
	  * envíe un caracter de confirmación luego de haber insertado el objeto */
	 Serial_Println("Inserte un peso conocido",'A');
	 while(HAL_UART_Receive(&huart1, &buf2, sizeof(buf2), 1) != HAL_OK){
	 }
	 val2=HX711_Read_Average(cantmuest); // Valor asociado al peso conocido

	 /* Solicitamos al usuario que retire el objeto. El programa se queda esperando a que el usuario envíe
	  * un caracter de confirmación luego de haber retirado el objeto */
	 Serial_Println("Retirar objeto",'A');
	 while(HAL_UART_Receive(&huart1, &buf2, sizeof(buf2), 1) != HAL_OK){
	 }

	 Serial_Println("Estableciendo tara...",'A');
	 tara=HX711_Read_Average(cantmuest); // Establecemos la tara del sensor

	 /* Hallamos la pendiente y la ordenada al origen de la recta de peso */
	 rectapeso.m=(P2-P1)/(val2-tara);
	 rectapeso.b=P1-rectapeso.m*tara;
	 Serial_Println("Escala de peso ajustada",'A');

}

/* Función que realiza una lectura del pluviómetro a partir del peso del agua en el recipiente */
float Leer_Pluviometro(void){

	uint32_t val;
	float peso;
	float pp;

	val=HX711_Read_Average(20); // Leemos un valor promedio de 20 muestras cada 1 ms
	peso=rectapeso.m*val+rectapeso.b; // Calculamos el peso del agua a partir del valor leído
	pp=peso/(pi*pow((d/2),2)); // Precipitación (en cm)
	pp=pp*10.0; // Precipitación (en mm)

	return pp;
}

/* Función que permite hacer un reajuste de la tara del sensor cuando se vacía el pluviómetro */
void HX711_Set_Tare(uint8_t cantmuest){

	uint32_t nuevatara; // Variable para guardar el nuevo valor leído del sensor cuando no hay peso colocado

	Serial_Println("Estableciendo tara...",'A');
	nuevatara=HX711_Read_Average(cantmuest); // Establecemos la nueva tara del sensor

	/* Hallamos la ordenada al origen de la nueva recta de peso. La pendiente no la recalculamos ya que suponemos
	 * que la pendiente de la recta sigue siendo la misma */
	rectapeso.b=P1-rectapeso.m*nuevatara;
	Serial_Println("Escala de peso ajustada",'A');

}
