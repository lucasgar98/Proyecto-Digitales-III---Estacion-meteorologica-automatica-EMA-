/*
 * BMP180.c
 *
 *  Created on: Oct 19, 2022
 *      Author: User
 */


#include "BMP180.h"

/* Direcciones de los bytes más significativos de los coeficientes de calibración*/
uint8_t DIR_MSB_COEF[11]={0xAA,0xAC,0xAE,0xB0,0xB2,0xB4,0xB6,0xB8,0xBA,0xBC,0xBE};
/* Direcciones de los bytes menos significativos de los coeficientes de calibración*/
uint8_t DIR_LSB_COEF[11]={0xAB,0xAD,0xAF,0xB1,0xB3,0xB5,0xB7,0xB9,0xBB,0xBD,0xBF};
BMP180_E2PROM calibcoef;

void BMP180_WriteRegister(uint8_t dirreg,uint8_t valreg){

	uint8_t buf2[2] = {dirreg,valreg};
	HAL_I2C_Master_Transmit(&hi2c2, I2C_ADDRESS_BMP180<<1, buf2, sizeof(buf2), 0xFFFF);
}

uint8_t BMP180_ReadRegister(uint8_t dir){

	uint8_t buf;
	HAL_I2C_Master_Transmit(&hi2c2, I2C_ADDRESS_BMP180<<1, &dir, sizeof(dir), 0xFFFF);
	HAL_I2C_Master_Receive(&hi2c2, I2C_ADDRESS_BMP180<<1, &buf, sizeof(buf), 0xFFFF);
	return buf;
}

short BMP180_Get_CalCoef1(uint8_t DIRMSB,uint8_t DIRLSB){

	uint8_t MSB,LSB;
	short calcoef;

    /* Leemos el byte más significativo del coeficiente*/
	MSB=BMP180_ReadRegister(DIRMSB);

	/* Leemos el byte menos significativo del coeficiente*/
	LSB=BMP180_ReadRegister(DIRLSB);

	calcoef=(MSB<<8)|LSB;

	return calcoef;

}

unsigned short BMP180_Get_CalCoef2(uint8_t dirmsb,uint8_t dirlsb){

	uint8_t MSB,LSB;
	unsigned short calcoef;

	 /*Leemos el byte más significativo del coeficiente*/
	MSB=BMP180_ReadRegister(dirmsb);

	 /*Leemos el byte menos significativo del coeficiente*/
	LSB=BMP180_ReadRegister(dirlsb);

	calcoef=(MSB<<8)|LSB;
	return calcoef;

}

void BMP180_Get_AllCalCoef(void){

	calibcoef.BMP180_AC1 = BMP180_Get_CalCoef1(DIR_MSB_COEF[0], DIR_LSB_COEF[0]);
	calibcoef.BMP180_AC2 = BMP180_Get_CalCoef1(DIR_MSB_COEF[1], DIR_LSB_COEF[1]);
	calibcoef.BMP180_AC3 = BMP180_Get_CalCoef1(DIR_MSB_COEF[2], DIR_LSB_COEF[2]);
	calibcoef.BMP180_AC4 = BMP180_Get_CalCoef2(DIR_MSB_COEF[3], DIR_LSB_COEF[3]);
	calibcoef.BMP180_AC5 = BMP180_Get_CalCoef2(DIR_MSB_COEF[4], DIR_LSB_COEF[4]);
	calibcoef.BMP180_AC6 = BMP180_Get_CalCoef2(DIR_MSB_COEF[5], DIR_LSB_COEF[5]);
	calibcoef.BMP180_B1 = BMP180_Get_CalCoef1(DIR_MSB_COEF[6], DIR_LSB_COEF[6]);
	calibcoef.BMP180_B2 = BMP180_Get_CalCoef1(DIR_MSB_COEF[7], DIR_LSB_COEF[7]);
	calibcoef.BMP180_MB = BMP180_Get_CalCoef1(DIR_MSB_COEF[8], DIR_LSB_COEF[8]);
	calibcoef.BMP180_MC = BMP180_Get_CalCoef1(DIR_MSB_COEF[9], DIR_LSB_COEF[9]);
	calibcoef.BMP180_MD = BMP180_Get_CalCoef1(DIR_MSB_COEF[10], DIR_LSB_COEF[10]);

}

long BMP180_Get_RawTemperature(void){

	uint8_t dircr=CR_ADDRESS; // Dirección del registro de control
	uint8_t valcr=CR_VALUE_TEMP; // Valor a escribir en el registro de control para obtener el dato de temp.
	uint8_t MSBTEMP,LSBTEMP; // Bytes correspondientes al dato de temperatura
	long UT;

	 /*Escribimos primero el registro de control con el valor 0x2E y esperamos 5 ms*/
	BMP180_WriteRegister(dircr,valcr);
	HAL_Delay(5); // Demora de 5 ms
	 /*Obtenemos el byte más significativo y el byte menos signficativo del dato de temperatura*/
	MSBTEMP=BMP180_ReadRegister(MSB_REG_ADDRESS);
	LSBTEMP=BMP180_ReadRegister(LSB_REG_ADDRESS);

	UT=(MSBTEMP<<8)|LSBTEMP;

	return UT;
}

long BMP180_Get_RawPressure(BMP180_OSS OSS){

	uint8_t DIRCR=CR_ADDRESS; // Dirección del registro de control
	uint8_t VALCR; // Valor a escribir en el registro de control para obtener el dato de presión
	uint8_t MSBPRES,LSBPRES,XLSBPRES; // Bytes correspondientes al dato de presión
	long UP;

/*	 El valor a escribir en el registro de control dependerá del modo de precisión en la conversión de la
	 * presión, el cual puede seleccionarse a partir del parámetro oss (oversampling setting). Este parámetro
	 * puede tomar cuatro valores diferentes:
	 * - oss = 0 --> Ultra low power (modo de ultra bajo consumo)
	 * - oss = 1 --> Standard (modo estándar)
	 * - oss = 2 --> High resolution (modo de alta resolución)
	 * - oss = 3 --> Ultra high resolution (modo de ultra alta resolución)*/

	switch(OSS){
	case 0:{
		VALCR=CR_VALUE_PRES_OSS0;
		BMP180_WriteRegister(DIRCR,VALCR);
		HAL_Delay(5); // Demora de 5 ms
	}
	break;
	case 1:{
		VALCR=CR_VALUE_PRES_OSS1;
		BMP180_WriteRegister(DIRCR,VALCR);
		HAL_Delay(8); // Demora de 8 ms
	}
	break;
	case 2:{
		VALCR=CR_VALUE_PRES_OSS2;
		BMP180_WriteRegister(DIRCR,VALCR);
		HAL_Delay(14); // Demora de 14 ms
	}
	break;
	case 3:{
		VALCR=CR_VALUE_PRES_OSS3;
		BMP180_WriteRegister(DIRCR,VALCR);
		HAL_Delay(26); // Demora de 26 ms
	}
	break;
	}

/*	 Obtenemos el byte más significativo y el byte menos signficativo del dato de presión. Si el modo
	 * de conversión elegido es de ultra alta resolución, el dato de presión ocupará 19 bits. Por esta razón,
	 * en este caso se debe obtener un byte adicional denominado XLSB*/
	MSBPRES=BMP180_ReadRegister(MSB_REG_ADDRESS);
	LSBPRES=BMP180_ReadRegister(LSB_REG_ADDRESS);
	XLSBPRES=BMP180_ReadRegister(XLSB_REG_ADDRESS);

	UP=((MSBPRES<<16)|(LSBPRES<<8)|XLSBPRES)>>(8-OSS);

	return UP;

}

float BMP180_Get_TrueTemperature(void){

	int32_t UTEMP,X1,X2,B5;
	short MC,MD;
	unsigned short AC5,AC6;
	int32_t TEMP;

	 /*Guardamos los coeficientes de calibración obtenidos en las variables definidas anteriormente*/
	AC5=calibcoef.BMP180_AC5;
	AC6=calibcoef.BMP180_AC6;
	MC=calibcoef.BMP180_MC;
	MD=calibcoef.BMP180_MD;

	UTEMP=BMP180_Get_RawTemperature(); // Obtenemos el dato de temperatura en crudo

/*	 Para obtener el dato real de temperatura en °C, es necesario realizar algunos cálculos auxiliares que
	 * están definidos en la hoja de datos del BMP180*/
	X1=(UTEMP-AC6)*AC5/32768;
	X2=MC*2048/(X1+MD);
	B5=X1+X2;
	TEMP=(B5+8)/16; // Dato de temperatura (en 0.1°C)

	return TEMP/10.0; // Retornamos la temperatura en °C
}

float BMP180_Get_TruePressure(BMP180_OSS oss){

	int32_t UTEMP,UPRES,X1,X2,X3,B3,B5,B6;
	uint32_t B4,B7;
	short AC1,AC2,AC3,B1,B2,MC,MD;
	unsigned short AC4,AC5,AC6;
	int32_t PRES; // Variable para guardar el valor de presión

	 /*Guardamos los coeficientes de calibración obtenidos en las variables definidas anteriormente*/
	AC1=calibcoef.BMP180_AC1;
	AC2=calibcoef.BMP180_AC2;
	AC3=calibcoef.BMP180_AC3;
	AC4=calibcoef.BMP180_AC4;
	AC5=calibcoef.BMP180_AC5;
	AC6=calibcoef.BMP180_AC6;
	B1=calibcoef.BMP180_B1;
	B2=calibcoef.BMP180_B2;
	MC=calibcoef.BMP180_MC;
	MD=calibcoef.BMP180_MD;

	UTEMP=BMP180_Get_RawTemperature(); // Obtenemos el dato de temperatura en crudo
	UPRES=BMP180_Get_RawPressure(oss); // Obtenemos el dato de presión en crudo

	X1=(UTEMP-AC6)*AC5/32768;
	X2=(MC*2048)/(X1+MD);
	B5=X1+X2;
	B6=B5-4000;
	X1=(B2*(B6*B6/4096))/2048;
	X2=AC2*B6/2048;
	X3=X1+X2;
	B3=(((AC1*4+X3)<<oss)+2)/4;
	X1=AC3*B6/8192;
	X2=(B1*(B6*B6/4096))/65536;
	X3=((X1+X2)+2)/4;
	B4=AC4*(uint32_t)(X3+32768)/32768;
	B7=((uint32_t)UPRES-B3)*(50000>>oss);
	if(B7<0x80000000){
		PRES=(B7*2)/B4;
	}
	else {
		PRES=(B7/B4)*2;
	}
	X1=(PRES/256)*(PRES/256);
	X1=(X1*3038)/65536;
	X2=(-7357*PRES)/65536;
	PRES=PRES+(X1+X2+3791)/16; // Valor de presión final (en Pa)

	return PRES/100.0; // Retornamos la presión en hPa

}
