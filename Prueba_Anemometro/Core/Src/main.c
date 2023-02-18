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
#define D 0.2098 // Diámetro de la trayectoria circular descrita por las cucharas del anemómetro (en m)
#define pi 3.141592654 // Número pi
#define N 60 // Número de muestras de vel. de viento a tomar en un intervalo de 5 min
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
float Obtener_Vel_Viento(void);
float Promedio_Muestras(float[]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Este programa permite tomar muestras de la velocidad del viento cada 10 s. El anemómetro funciona gracias
 * a un sensor detector de campo magnético por efecto Hall, el cual detecta el campo magnético generado por
 * un imán de neodimio ubicado sobre la pieza móvil. Cuando el sensor detecta el campo magnético, su salida
 * pasa de 1 a 0, lo que constituye un flanco descendente. A medida que la pieza móvil gira, el imán pasa
 * varias veces por la posición del sensor, por lo que se generan distintos pulsos que pueden ser detectados
 * mediante una interrupción por flanco descendente.
 * La velocidad del viento se mide contando la cantidad de pulsos generados en un intervalo de tiempo
 * determinado. En este caso, se contarán la cantidad de pulsos en 10 s, y luego se hará la conversión de
 * velocidad angular a velocidad lineal.
 * Para generar las demoras necesarias para el conteo de pulsos, se utilizará el timer TIM2. Se configura
 * el canal CH2 en modo output compare. El valor de autorecarga del contador (valor máximo) será de 60.000,
 * y la frecuencia de reloj del sistema es de 12 MHz. Para que el contador realice 60.000 cuentas en 5 min
 * (300 s), el prescaler debe valer 60.000. De esta manera, el contador realiza 200 cuentas en 1 s.
 * El canal 2 del TIM2 genera la demora necesaria para tomar las muestras de la velocidad del viento. Al
 * iniciar el programa, comienza el conteo de pulsos, y dicho conteo se realiza durante 10 s. Esto significa
 * que el registro de capture/compare del CH2 debe ser cargado inicialmente con el valor 1999, para que la
 * interrupción por OC se produzca al llegar a las 2000 cuentas. Los pulsos se cuentan a partir de las int.
 * por flanco descendente recibidas en el pin correspondiente.
 * Una vez que finaliza el conteo de los pulsos, se debe actualizar el valor del registro de capture/
 * compare del CH2, para que dicho canal genere otra interrupción por OC dentro de 10 s. Por lo tanto, al
 * contenido del registro de CC del CH2 se le debe sumar 2000. Al mismo tiempo, se deben deshabilitar las int.
 * externas para poder realizar el cálculo de la vel. del viento, y una vez finalizado el cálculo se vuelven
 * a habilitar dichas interrupciones.
 * Finalmente, cada 5 min se calculará el promedio de las muestras de velocidad de viento tomadas en los
 * últimos 5 min.
 */

_Bool flag_pulsos=0; // Variable bandera que se activa cuando termina el conteo de pulsos
uint16_t contpulsos=0; // Variable para contar los pulsos generados por el anemómetro en 1 s
_Bool flag_muestras=0; // Variable bandera que se activa cada 5 min cuando el contador llega a su valor máximo

void HAL_TIM_OC_DelayElapsedCallback (TIM_HandleTypeDef * htim){

	uint16_t pulse;
	/*switch(htim->Channel){
	case HAL_TIM_ACTIVE_CHANNEL_1:{
		flag_muestras=1;
		pulse=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,(pulse+(60000)));
	}
	break;
	case HAL_TIM_ACTIVE_CHANNEL_2:{
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);
		flag_pulsos=1;
		pulse=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,(pulse+(2000)));
	}
	break;
	} */
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);
		flag_pulsos=1;
		pulse=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

		if(__HAL_TIM_GET_AUTORELOAD(htim)>(pulse+1000)){
			__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,(pulse+(1000)));
		}
		else {
			__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,999);
		}
	}

	/*switch(htim->Channel){
	case HAL_TIM_ACTIVE_CHANNEL_1:{
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);
		flag_pulsos=1;
		pulse = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,(pulse+(2200)));
		pulse=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,(pulse+(2000)));
	}
	break;
	case HAL_TIM_ACTIVE_CHANNEL_2:{
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	}
	break;
	}*/

}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim){

	flag_muestras=1;
}

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){

	contpulsos++; // Incrementamos el contador de pulsos cada vez que recibimos una int por flanco descendente
}

float Obtener_Vel_Viento(void){

	float velangular; // Velocidad angular (en rad/s)
	float vellineal; // Velocidad lineal (en m/s)
	float velviento; // Velocidad lineal del viento (en km/h)

	/* Realizamos la conversión de velocidad angular a velocidad lineal */
	velangular=(contpulsos/5.0)*2.0*pi;
	vellineal=velangular*D/2;
	velviento=vellineal*3.6; // Convertimos de m/s a km/h

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  float vel[N]={0}; // Variable para guardar las muestras de velocidad del viento (en km/h)
  float velmed; // Velocidad del viento media (en km/h)
  uint8_t cantmuestras=0; // Cantidad de muestras de vel. del viento
  uint8_t buffer[40];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(flag_muestras==1){
		  flag_muestras=0;
		  velmed=Promedio_Muestras(vel);
		  sprintf((char*)buffer,"*AVel del viento media: %2.2f km/h\n*",velmed);
		  HAL_UART_Transmit(&huart1, buffer, strlen((const char*)buffer),100);
		  cantmuestras=0;

	  }
	  if(flag_pulsos==1){
		  flag_pulsos=0;
		  vel[cantmuestras]=Obtener_Vel_Viento();
		  sprintf((char*)buffer,"*ACant. de pulsos: %d\n*",contpulsos);
		  HAL_UART_Transmit(&huart1, buffer, strlen((const char*)buffer),100);
		  sprintf((char*)buffer,"*AVel del viento: %2.2f km/h\n*",vel[cantmuestras]);
		  HAL_UART_Transmit(&huart1, buffer, strlen((const char*)buffer),100);
		  contpulsos=0;
		  cantmuestras++;
		  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 60000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 59999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  //HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1); // Inicializamos el CH1 del TIM2 en modo output compare
  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2); // Inicializamos el CH2 del TIM2 en modo output compare
  /* USER CODE END TIM2_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
