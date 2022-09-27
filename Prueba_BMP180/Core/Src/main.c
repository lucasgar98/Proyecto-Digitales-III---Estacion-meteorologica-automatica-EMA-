/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Definimos una variable tipo enumeración para guardar los valores que puede tomar el parámetro oss
 * (oversampling setting) */
typedef enum BMP180_OSS {
	BMP180_LOW, BMP180_STANDARD, BMP180_HIGH, BMP180_ULTRA,
} BMP180_OSS;
/* Definimos una variable tipo estructura para guardar los coeficientes de calibración */
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
float BMP180_Get_TrueTemperature(void);
float BMP180_Get_TruePressure(BMP180_OSS);
void BMP180_Get_AllCalCoef(void);
long BMP180_Get_RawTemperature(void);
long BMP180_Get_RawPressure(uint8_t);
short BMP180_Get_CalCoef1(uint8_t,uint8_t);
unsigned short BMP180_Get_CalCoef2(uint8_t,uint8_t);
void BMP180_WriteRegister(uint8_t,uint8_t);
uint8_t BMP180_ReadRegister(uint8_t);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Direcciones de los bytes más significativos de los coeficientes de calibración */
uint8_t DIR_MSB_COEF[11]={0xAA,0xAC,0xAE,0xB0,0xB2,0xB4,0xB6,0xB8,0xBA,0xBC,0xBE};
/* Direcciones de los bytes menos significativos de los coeficientes de calibración */
uint8_t DIR_LSB_COEF[11]={0xAB,0xAD,0xAF,0xB1,0xB3,0xB5,0xB7,0xB9,0xBB,0xBD,0xBF};
BMP180_E2PROM calibcoef;

void BMP180_WriteRegister(uint8_t dirreg,uint8_t valreg){

	uint8_t buf2[2] = {dirreg,valreg};
	HAL_I2C_Master_Transmit(&hi2c2, I2C_ADDRESS_BMP180<<1, buf2, sizeof(buf2), 0xFFFF);
}

uint8_t BMP180_ReadRegister(uint8_t dir){

	uint8_t buf;
	HAL_I2C_Master_Transmit(&hi2c2, I2C_ADDRESS_BMP180<<1, &dir, sizeof(dir), 0xFFFF);
	HAL_I2C_Master_Receive(&hi2c2, I2C_ADDRESS_BMP180<<1, &buf, sizeof(buf), 0xFFF);
	return buf;
}

short BMP180_Get_CalCoef1(uint8_t DIRMSB,uint8_t DIRLSB){

	uint8_t MSB,LSB;
	short calcoef;

	/* Leemos el byte más significativo del coeficiente */
	MSB=BMP180_ReadRegister(DIRMSB);

	/* Leemos el byte menos significativo del coeficiente */
	LSB=BMP180_ReadRegister(DIRLSB);

	calcoef=(MSB<<8)|LSB;

	return calcoef;

}

unsigned short BMP180_Get_CalCoef2(uint8_t dirmsb,uint8_t dirlsb){

	uint8_t MSB,LSB;
	unsigned short calcoef;

	/* Leemos el byte más significativo del coeficiente */
	MSB=BMP180_ReadRegister(dirmsb);

	/* Leemos el byte menos significativo del coeficiente */
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

	/* Escribimos primero el registro de control con el valor 0x2E y esperamos 5 ms */
	BMP180_WriteRegister(dircr,valcr);
	HAL_Delay(5); // Demora de 5 ms
	/* Obtenemos el byte más significativo y el byte menos signficativo del dato de temperatura */
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

	/* El valor a escribir en el registro de control dependerá del modo de precisión en la conversión de la
	 * presión, el cual puede seleccionarse a partir del parámetro oss (oversampling setting). Este parámetro
	 * puede tomar cuatro valores diferentes:
	 * - oss = 0 --> Ultra low power (modo de ultra bajo consumo)
	 * - oss = 1 --> Standard (modo estándar)
	 * - oss = 2 --> High resolution (modo de alta resolución)
	 * - oss = 3 --> Ultra high resolution (modo de ultra alta resolución) */

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

	/* Obtenemos el byte más significativo y el byte menos signficativo del dato de presión. Si el modo
	 * de conversión elegido es de ultra alta resolución, el dato de presión ocupará 19 bits. Por esta razón,
	 * en este caso se debe obtener un byte adicional denominado XLSB */
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

	/* Guardamos los coeficientes de calibración obtenidos en las variables definidas anteriormente */
	AC5=calibcoef.BMP180_AC5;
	AC6=calibcoef.BMP180_AC6;
	MC=calibcoef.BMP180_MC;
	MD=calibcoef.BMP180_MD;

	UTEMP=BMP180_Get_RawTemperature(); // Obtenemos el dato de temperatura en crudo

	/* Para obtener el dato real de temperatura en °C, es necesario realizar algunos cálculos auxiliares que
	 * están definidos en la hoja de datos del BMP180 */
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

	/* Guardamos los coeficientes de calibración obtenidos en las variables definidas anteriormente */
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  BMP180_Get_AllCalCoef(); // Obtenemos todos los coeficientes de calibración

  float temp,pres;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  temp=BMP180_Get_TrueTemperature();
	  pres=BMP180_Get_TruePressure(BMP180_STANDARD);
	  HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
