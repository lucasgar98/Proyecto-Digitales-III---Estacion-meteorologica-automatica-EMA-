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
/*
#include "stdio.h"
#include "string.h"
#include "math.h"
*/
#include "veleta3.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*enum DIR_VIENTO{
	N = 0,
	NE = -45,
	E = -90,
	SE = -135,
	S = 180,
	SW = 135,
	W = 90,
	NW = 45,
};*/
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define positivo 0
#define negativo 1
#define M 120
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
_Bool flag_veleta=0; // Variable bandera que se activa cada 500 ms para tomar muestras de la tensión analógica de la veleta
_Bool flag_muestras=0; // Variable bandera que se activa cada 1 ms
_Bool flag_adquisicion=0; // Variable bandera que se activa cada 1 min
_Bool flag_conversion=0; // Variable bandera que se activa al finalizar la conversión
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/*float Prom_Muestras_Veleta(uint8_t);
uint8_t Detectar_Angulo(float,_Bool);
float Promedio_Angulo(uint8_t[]);
uint8_t Contar_Signos(_Bool[],_Bool);
int16_t Obtener_Dir_Viento(float,uint8_t,uint8_t);
void Enviar_Dir_Viento(int16_t);*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){

	uint16_t pulse;
	if(htim->Instance==TIM2){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			//HAL_NVIC_DisableIRQ(EXTI0_IRQn);
			//flag_pulsos=1;
			flag_veleta=1;
			pulse=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

			if(__HAL_TIM_GET_AUTORELOAD(htim)>(pulse+500)){
				__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,(pulse+(500)));
			}
			else {
				__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,499);
			}
		}
	}
	if(htim->Instance==TIM3){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			//HAL_NVIC_DisableIRQ(EXTI0_IRQn);
			//flag_pulsos=1;
			flag_muestras=1;
			pulse=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

			if(__HAL_TIM_GET_AUTORELOAD(htim)>(pulse+60)){
				__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,(pulse+(60)));
			}
			else {
				__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,59);
			}
		}
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim->Instance==TIM2){
		flag_adquisicion=1;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	flag_conversion=1;
}

/* Esta función toma muestras de tensión analógica cada 1 ms, y luego calcula el promedio
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

uint8_t Detectar_Angulo(float volprom, _Bool tipo){

	uint8_t angulo;
	_Bool signo;

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
}*/
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  float volveleta;
  uint8_t angdir[M];
  _Bool sigang[M];
  uint8_t cant=0;
  float promangdir;
  uint8_t cantsigpos,cantsigneg;
  /* Dirección del viento obtenida como un promedio de las direcciones del viento muestreadas en 1 min */
  int16_t dirv;

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(flag_veleta==1){
		  flag_veleta=0;
		  volveleta=Prom_Muestras_Veleta(50);
		  angdir[cant]=Detectar_Angulo(volveleta,0);
		  sigang[cant]=Detectar_Angulo(volveleta,1);
		  cant++;
	  }

	  if(flag_adquisicion==1){
		  flag_adquisicion=0;
		  promangdir=Promedio_Angulo(angdir);
		  cantsigpos=Contar_Signos(sigang,positivo);
		  cantsigneg=Contar_Signos(sigang,negativo);
		  dirv=Obtener_Dir_Viento(promangdir,cantsigpos,cantsigneg);
		  Enviar_Dir_Viento(dirv);
		  cant=0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48000;
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
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 800;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 59999;
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
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 59;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

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
