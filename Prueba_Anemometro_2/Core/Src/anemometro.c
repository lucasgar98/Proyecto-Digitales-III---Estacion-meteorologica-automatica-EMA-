/*
 * anemometro.c
 *
 *  Created on: 9 nov. 2022
 *      Author: User
 */

#include "anemometro.h"

float Obtener_Vel_Viento(void){

	float velviento; // Velocidad del viento (en km/h)

	/* Para obtener la velocidad del viento, se debe tener en cuenta que una velocidad de 2.4 km/h equivale
	 * a un pulso por segundo, y que el anemómetro emite dos pulsos por revolución. En este caso, se cuentan
	 * los pulsos emitidos por el anemómetro en un intervalo de tiempo de 5 s. Sabiendo que 1 pulso por segundo
	 * equivale a 2.4 km/h, entonces 5 pulsos en 5 segundos corresponden a una velocidad del viento de 2.4 km/h.
	 * Por lo tanto dividimos la cantidad de pulsos por la longitud del intervalo (5 s) y luego multiplicamos
	 * por 2.4 */

	velviento=(contpulsos/INT_PULSOS)*FACTOR_CONV;

	return velviento;
}

float Promedio_Muestras(float v[]){

	float suma=0;
	float prom;
	for(int i=0;i<N;i++){
		suma=suma+v[i];
	}
	prom=suma/N;

	return prom;
}
