/*
 * viento.c
 *
 *  Created on: 28 ene. 2023
 *      Author: User
 */

#include "viento.h"

/* Esta función toma muestras de tensión analógica cada 1 ms, y luego calcula el promedio */
float Prom_Muestras_Veleta(uint8_t cantmuestras){

	  uint16_t vol[cantmuestras]; // Tensión analógica muestreada en mV
	  uint32_t sum=0; // Variable que suma las muestras
	  float prom;
	  uint8_t cant=0;

	  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1); // Iniciamos el TIM3 en modo comparación de salida con interrupción
	  /* Cada 1 ms, leemos un valor de tensión analógica de la veleta */
	  while(cant<cantmuestras){
		  if(flag_muestras==1){
			  flag_muestras=0; // Reseteamos la bandera
			  HAL_ADC_Start_IT(&hadc1); // Iniciamos la conversión del ADC en modo interrupción
		  }
		  else {
			  if(flag_conversion==1){
				  flag_conversion=0; // Reseteamos la bandera de conversión
				  // Hallamos la tensión equivalente a partir del valor codificado en decimal
			      vol[cant]=HAL_ADC_GetValue(&hadc1)*3300/4095;
			      cant++;
			  }
		  }
	  }
	  HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_1); // Detenemos el TIM3 en modo comparación de salida
	  /* Sumamos todas las muestras de tensión analógica */
	  for(int i=0;i<cantmuestras;i++){
		  sum=sum+vol[i];
	  }
	  prom=sum/cantmuestras; // Calculamos el promediod de muestras

	  return prom;
}

/* Función que le asigna un ángulo a la dirección del viento en función de la tensión analógica muestreada */
uint8_t Detectar_Angulo(float volprom, _Bool tipo){

	uint8_t angulo=0;
	_Bool signo=0;

	/* Asociamos a cada valor de tensión analógica muestreado una dirección determinada
	 * - Entre 0 y 60 mV --> E (incluye ENE y ESE)
	 * - Entre 70 y 170 mV --> SE (incluye SSE)
	 * - Entre 185 y 300 mV --> S (incluye SSW)
	 * - Entre 380 y 600 mV --> NE (incluye NNE)
	 * - Entre 700 y 1000 mV --> SW (incluye WSW)
	 * - Entre 1050 y 1650 mV --> N (incluye NNW)
	 * - Entre 1700 y 2500 mV --> NW (incluye WNW)
	 * - Entre 2800 y 3300 mV --> W
	 */

	if(volprom<60){
		angulo=E;
		signo=positivo;
	}
	else if(70<volprom && volprom<170){
		angulo=SE;
		signo=positivo;
	}
	else if(185<volprom && volprom<300){
		angulo=S;
		signo=positivo;
	}
	else if(380<volprom && volprom<600){
		angulo=NE;
		signo=positivo;
	}
	else if(700<volprom && volprom<1000){
		angulo=fabs(SW);
		signo=negativo;
	}
	else if(1050<volprom && volprom<1650){
		angulo=N;
		signo=positivo;
	}
	else if(1700<volprom && volprom<2500){
		angulo=fabs(NW);
		signo=negativo;
	}
	else if(2800<volprom){
		angulo=fabs(W);
		signo=negativo;
	}

	if(tipo==0){
		return angulo;
	}
	else return signo;

}

/* Rutina que realiza el promedio de ángulo asociado a cada una de las muestras de dirección del viento */
float Promedio_Angulo(uint8_t angulo[]){

	float promangulo;
	uint16_t sumangulo=0;

	for(int i=0;i<N1;i++){
		sumangulo=sumangulo+angulo[i];
	}
	promangulo=sumangulo/N1;
	return promangulo;
}

/* Rutina que cuenta los signos de los ángulos asociados a las muestras de dirección del viento */
uint8_t Contar_Signos(_Bool signo[],_Bool tipo){

	uint8_t cantsignospos=0;
	uint8_t cantsignosneg=0;

	/* Recorremos el vector que guarda los signos para contar los signos positivos y negativos */
	for(int i=0;i<N1;i++){
		if(signo[i]==positivo){
			cantsignospos++;
		}
		else cantsignosneg++;
	}

	if(tipo==positivo){
		return cantsignospos;
	}
	else return cantsignosneg;

}

/* Rutina que obtiene la dirección del viento a partir del ángulo promedio y de la cantidad de signos positivos
 * y negativos de los ángulos */
DIR_VIENTO Obtener_Dir_Viento(float angprom, uint8_t contsigpos, uint8_t contsigneg){

	DIR_VIENTO dirviento=0;
	float angdirv;
	/* Detectamos la dirección del viento a partir del ángulo promedio y los signos
	 * - Ángulo entre -22.5 y +22.5 --> Dirección N
	 * - Ángulo entre +22.5 y +67.5 --> Dirección NE
	 * - Ángulo entre +67.5 y +112.5 --> Dirección E
	 * - Ángulo entre +112.5 y +157.5 --> Dirección SE
	 * - Ángulo entre +157.5 y +180 o entre -180 y -157.5 --> Dirección S
	 * - Ángulo entre -157.5 y -112.5 --> Dirección SW
	 * - Ángulo entre -112.5 y -67.5 --> Dirección W
	 * - Ángulo entre -67.5 y -22.5 --> Dirección NW
	 */

	if(angprom>=0 && angprom<22.5){
		dirviento=N;
	}
	else if(angprom>=22.5 && angprom<67.5){
		if(contsigpos>contsigneg){
			dirviento=NE;
		}
		else dirviento=NW;
	}
	else if(angprom>=67.5 && angprom<112.5){
		if(contsigpos>contsigneg){
			dirviento=E;
		}
		else dirviento=W;
	}
	else if(angprom>=112.5 && angprom<157.5){
		if(contsigpos>contsigneg){
			dirviento=SE;
		}
		else dirviento=SW;
	}
	else if(angprom>=157.5 && angprom<=180){
		dirviento=S;
	}

	return dirviento;
}

/* Función que envía la dirección del viento al módulo Bluetooth a través del puerto serie (UART) */
void Enviar_Dir_Viento(DIR_VIENTO dir){

	switch(dir){
	case N:{
		Enviar_Caracter("N",'D');
	}
	break;
	case NE:{
		Enviar_Caracter("NE",'D');
	}
	break;
	case E:{
		Enviar_Caracter("E",'D');
	}
	break;
	case SE:{
		Enviar_Caracter("SE",'D');
	}
	break;
	case S:{
		Enviar_Caracter("S",'D');
	}
	break;
	case SW:{
		Enviar_Caracter("SW",'D');
	}
	break;
	case W:{
		Enviar_Caracter("W",'D');
	}
	break;
	case NW:{
		Enviar_Caracter("NW",'D');
	}
	break;
	}
}

/* Función que guarda el dato de dirección del viento en la memoria SD */
void Escribir_Dir_Viento(DIR_VIENTO dir, char carfinal[]){

	uint8_t buffer[5];
	uint32_t output;
	char str[5];

	switch(dir){
	case N:{
		strcpy(str,"N");
		strcat(str,carfinal);
	}
	break;
	case NE:{
		strcpy(str,"NE");
		strcat(str,carfinal);
	}
	break;
	case E:{
		strcpy(str,"E");
		strcat(str,carfinal);
	}
	break;
	case SE:{
		strcpy(str,"SE");
		strcat(str,carfinal);
	}
	break;
	case S:{
		strcpy(str,"S");
		strcat(str,carfinal);
	}
	break;
	case SW:{
		strcpy(str,"SW");
		strcat(str,carfinal);
	}
	break;
	case W:{
		strcpy(str,"W");
		strcat(str,carfinal);
	}
	break;
	case NW:{
		strcpy(str,"NW");
		strcat(str,carfinal);
	}
	break;
	}

	sprintf((char*)buffer,"%s",str);

	if (f_write(&USERFile,buffer,strlen((const char*)buffer),(void*)&output)==FR_OK){
		Serial_Print("R0G255B0",'C'); // Encendemos un indicador verde si el archivo se escribió correctamente
		if(strcmp(carfinal,"\n")==0){
			 if (f_sync(&USERFile)==FR_OK){
				 f_close(&USERFile);
			 }
		}
	}
	else {
		Serial_Print("R255G0B0",'C'); // Encendemos un indicador rojo si el archivo no se pudo escribir
	}

}

/* Realiza el promedio de las muestras de velocidad del viento */
float Promedio_Muestras_Anemometro(float v[]){

	float suma=0;
	float prom;

	for(int i=0;i<N1;i++){
		suma=suma+v[i];
	}
	prom=suma/N1;

	return prom;
}

/* Toma muestras de la velocidad y dirección del viento cada cierto tiempo (generalmente 3 o 5 segundos). Los datos
 * muestreados se guardan en una variable estructura */
void Tomar_Muestras_Viento(VIENTO *dirvel){

	float volveleta;
	float velviento;

	if(flag_viento==1){
		flag_viento=0; // Reseteamos la variable bandera
		/* Tomamos primero una muestra de la dirección del viento */
		volveleta=Prom_Muestras_Veleta(50); // Medimos la tensión analógica de la veleta (prom. 50 muestras cada 1 ms)
		dirvel->angdir[cantmuestviento]=Detectar_Angulo(volveleta,0); // Detectamos el ángulo de la dir. del viento
		dirvel->sigdir[cantmuestviento]=Detectar_Angulo(volveleta,1); // Detectamos el signo del ángulo
		/* Ahora muestreamos la velocidad del viento */
		velviento=(contpulsos/INT_PULSOS)*FACTOR_CONV; // Fórmula para hallar la velocidad del viento
		dirvel->velv[cantmuestviento]=velviento; // Guardamos la vel. del viento en la variable correspondiente
		contpulsos=0; // Reseteamos la variable contador
		cantmuestviento++; // Incrementamos la variable que cuenta las muestras de dir. y vel. del viento
		HAL_NVIC_EnableIRQ(EXTI0_IRQn); // Habilitamos las interrupciones externas
	}

}
