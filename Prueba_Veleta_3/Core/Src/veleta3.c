/*
 * veleta3.c
 *
 *  Created on: 9 nov. 2022
 *      Author: User
 */

#include "veleta3.h"

/* Esta función toma muestras de tensión analógica cada 1 ms, y luego calcula el promedio */
float Prom_Muestras_Veleta(uint8_t cantmuestras){

	  uint16_t vol[cantmuestras]; // Tensión analógica muestreada en mV
	  uint32_t sum=0;
	  float prom;
	  uint8_t cant=0;

	  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
	  while(cant<cantmuestras){
		  if(flag_muestras==1){
			  flag_muestras=0;
			  HAL_ADC_Start_IT(&hadc1);
		  }
		  else {
			  if(flag_conversion==1){
				  flag_conversion=0;
			      vol[cant]=HAL_ADC_GetValue(&hadc1)*3300/4095;
			      cant++;
			  }
		  }
	  }
	  HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_1);
	  for(int i=0;i<cantmuestras;i++){
		  sum=sum+vol[i];
	  }
	  prom=sum/cantmuestras;

	  return prom;
}

ANG_DIRV Detectar_Angulo(float volprom){

	uint8_t angulo;
	_Bool signo;
	ANG_DIRV angdirv;

	if(volprom<100){
		angulo=fabs(E);
		signo=negativo;
	}
	else if(110<volprom && volprom<200){
		angulo=fabs(SE);
		signo=negativo;
	}
	else if(210<volprom && volprom<350){
		angulo=fabs(S);
		signo=positivo;
	}
	else if(400<volprom && volprom<700){
		angulo=fabs(NE);
		signo=negativo;
	}
	else if(800<volprom && volprom<1100){
		angulo=fabs(SW);
		signo=positivo;
	}
	else if(1200<volprom && volprom<1800){
		angulo=fabs(N);
		signo=positivo;
	}
	else if(2000<volprom && volprom<2500){
		angulo=fabs(NW);
		signo=positivo;
	}
	else if(3000<volprom){
		angulo=fabs(W);
		signo=positivo;
	}

	angdirv.ang[]

	if(tipo == 0){
		return angulo;
	}
	else return signo;
}

float Promedio_Angulo(uint8_t angulo[]){

	float promangulo;
	uint32_t sumangulo=0;

	for(int i=0;i<M;i++){
		sumangulo=sumangulo+angulo[i];
	}
	promangulo=sumangulo/M;
	return promangulo;
}

uint8_t Contar_Signos(_Bool signo[],_Bool tipo){

	uint8_t cantsignospos=0;
	uint8_t cantsignosneg=0;

	for(int i=0;i<M;i++){
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

int16_t Obtener_Dir_Viento(float angprom, uint8_t contsigpos, uint8_t contsigneg){

	int16_t dirviento;

	if(angprom>=0 && angprom<22.5){
		dirviento=N;
	}
	else if(angprom>22.5 && angprom<67.5){
		if(contsigpos>contsigneg){
			dirviento=NW;
		}
		else dirviento=NE;
	}
	else if(angprom>67.5 && angprom<112.5){
		if(contsigpos>contsigneg){
			dirviento=W;
		}
		else dirviento=E;
	}
	else if(angprom>112.5 && angprom<157.5){
		if(contsigpos>contsigneg){
			dirviento=SW;
		}
		else dirviento=SE;
	}
	else if(angprom>157.5 && angprom<=180){
		dirviento=S;
	}

	return dirviento;
}

void Enviar_Dir_Viento(int16_t dir){

	uint8_t buf[30];

    //sprintf((char*)buffer,"*ATensión analógica: %4.1f mV \n*",vprom);
	//HAL_UART_Transmit(&huart1, buffer, strlen((const char*)buffer),100);

	switch(dir){
	case N:{
	    sprintf((char*)buf,"*ADireccion del viento: N\n*");
		HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	}
	break;
	case NE:{
	    sprintf((char*)buf,"*ADireccion del viento: NE\n*");
	    HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	}
	break;
	case E:{
	    sprintf((char*)buf,"*ADireccion del viento: E\n*");
	    HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	}
	break;
	case SE:{
	    sprintf((char*)buf,"*ADireccion del viento: SE\n*");
	    HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	}
	break;
	case S:{
	    sprintf((char*)buf,"*ADireccion del viento: S\n*");
	    HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	}
	break;
	case SW:{
	    sprintf((char*)buf,"*ADireccion del viento: SW\n*");
	    HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	}
	break;
	case W:{
	    sprintf((char*)buf,"*ADireccion del viento: W\n*");
	    HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	}
	break;
	case NW:{
	    sprintf((char*)buf,"*ADireccion del viento: NW\n*");
	    HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	}
	break;
	}
}
