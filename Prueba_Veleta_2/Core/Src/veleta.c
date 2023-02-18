/*
 * veleta.c
 *
 *  Created on: Oct 25, 2022
 *      Author: User
 */

#include "veleta.h"

/* Esta función toma muestras de tensión analógica cada 1 ms, y luego calcula el promedio */
float Prom_Muestras_Veleta(uint8_t cantmuestras){

	  uint16_t vol[cantmuestras]; // Tensión analógica muestreada en mV
	  uint32_t sum=0;
	  float prom;
	  uint8_t cant=0;

	  HAL_TIM_Base_Start_IT(&htim4);
	  while(cant<cantmuestras){
		  if(flag_muestreo==1){
			  flag_muestreo=0;
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
	  HAL_TIM_Base_Stop_IT(&htim4);
	  for(int i=0;i<cantmuestras;i++){
		  sum=sum+vol[i];
	  }
	  prom=sum/cantmuestras;

	  return prom;
}

uint8_t Detectar_Dir_Viento(float volprom){

	uint8_t dirv;

	if(volprom<100){
		dirv=E;
	}
	else if(110<volprom && volprom<200){
		dirv=SE;
	}
	else if(210<volprom && volprom<350){
		dirv=S;
	}
	else if(400<volprom && volprom<700){
		dirv=NE;
	}
	else if(800<volprom && volprom<1100){
		dirv=SW;
	}
	else if(1200<volprom && volprom<1800){
		dirv=N;
	}
	else if(2000<volprom && volprom<2500){
		dirv=NW;
	}
	else if(3000<volprom){
		dirv=W;
	}

	return dirv;
}

void Enviar_Dir_Viento(uint8_t dir){

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
