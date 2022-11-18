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
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_ADDRESS_AHT10 0x38
#define TRIGGER_MEASUREMENT 0xAC
#define DATA0 0x33
#define DATA1 0x00
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
float tcelsius, hr; // Temperatura (en °C) y humedad relativa (%)
_Bool flag_muestras=0; // Variable bandera para iniciar la toma de muestras de la temp. y humedad cada 1 s

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void AHT10_Read(void);
void Serial_Print(char[],char,void*);
void Serial_Println(char[],char,float);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Este programa permite mandar los datos de temperatura y humedad leídos del sensor AHT10 a través del módulo
 * Bluetooth, para poder visualizar esos datos en una app del teléfono llamada Bluetooth Electronics */

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim){

	flag_muestras=1;
}

/* Esta función permite leer datos de temperatura y humedad del sensor AHT10. Para que el sensor
 * comience con las mediciones, es necesario enviar un comando de disparo de medición (trigger
 * measurement) junto con dos bytes de datos DATA0 y DATA1. El sensor tardará un tiempo en
 * realizar la medición y recolectar los datos (más de 75 ms). Transcurrido ese tiempo,
 * los datos de temperatura y humedad son recibidos, repartidos en 5 bytes, junto con un byte que
 * indica el estado del sensor */
void AHT10_Read(){

	/* Definimos el comando de disparo de medición y lo enviamos a través de I2C1 */
	uint8_t triggerdata[3] = {TRIGGER_MEASUREMENT,DATA0,DATA1};
	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS_AHT10<<1, triggerdata, sizeof(triggerdata), 0xFFFF);
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
	if(HAL_I2C_Master_Receive(&hi2c1, I2C_ADDRESS_AHT10<<1, bufdata, sizeof(bufdata), 0xFFFF) == HAL_OK){

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

void Serial_Print(char msj[], char rxchar, void *dat){

	uint8_t buf[40];
	char str1[40] = {'*',rxchar};
	char str2[2] = {'*'};

	strcat(str1,msj);
	strcat(str1,str2);

	sprintf((char*)buf,str1);
	HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
}

void Serial_Println_Float(char msj[], char rxchar, float dat){

	uint8_t buf[40];
	char str1[40] = {'*',rxchar};
	char str2[3] = "\n*";

	strcat(str1,msj);
	strcat(str1,str2);

	sprintf((char*)buf,str1,dat);
	HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //HM10_Init_AT_Commands();
  //uint8_t buffer[30];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(flag_muestras==1){
			flag_muestras=0;
			AHT10_Read();
			Serial_Println_Float("Temperatura: %2.1f °C",'A',tcelsius);
			Serial_Println_Float("Humedad: %2.1f %",'A',hr);
			//sprintf((char*)buffer,"*ATemperatura: %2.1f\n*",tcelsius);
			//HAL_UART_Transmit(&huart1, buffer, strlen((const char*)buffer),100);
			//sprintf((char*)buffer,"*AHumedad: %2.1f\n*",hr);
			//HAL_UART_Transmit(&huart1, buffer, strlen((const char*)buffer),100);
		}
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 750;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 64000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
	HAL_TIM_Base_Start_IT(&htim3); // Inicializamos la generación de base de tiempos del TIM3 en modo no bloqueante
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
