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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AHT10.h"
#include "BMP180.h"
#include "HX711.h"
#include "RTC.h"
#include "serial.h"
#include "SPISD.h"
#include "viento.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Definimos una estructura para guardar los datos meteorológicos adquiridos */
typedef struct METEO_DATA {
	float temp;
	float humrel;
	float pres;
	float velviento;
	DIR_VIENTO dirviento;
	float precip;
} METEO_DATA;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

EXTI_HandleTypeDef hexti; // Variable para limpiar las interrupciones externas pendientes

/* Variables utilizadas para la configuración de la memoria SD */
SPISD spisd;
SPISD *mainSD = &spisd;
uint8_t Sector0[516];

/* Variables tipo bandera */
_Bool flag_muestras=0; // Variable bandera que se pone en 1 cuando interrumpe por OC el CH1 del TIM3, cada 1 ms
_Bool flag_conversion=0; // Variable bandera que se activa cuando finaliza la conversión del ADC
_Bool flag_viento=0; // Variable bandera que se activa cada 3 s para tomar muestras de la vel. y dir. del viento

/* Variables tipo contador */
uint16_t contpulsos=0; // Variable para contar los pulsos generados por el anemómetro en 3 s
uint8_t contminutos=0; // Variable para contar los minutos
uint8_t cantmuestviento=0; // Variable para contar las muestras de dirección y velocidad del viento

/* Variables adicionales */
VIENTO dirvelviento; // Variable tipo estructura para guardar las muestras de dirección y velocidad del viento
FECHA fechaactual; // Variable estructura para guardar la fecha actual leída del RTC
HORA horaactual; // Variable estructura para guardar la hora actual leída del RTC
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Adquirir_Datos(METEO_DATA*);
void Enviar_Datos(METEO_DATA*);
void Almacenar_Datos(METEO_DATA*);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Rutina que se ejecuta cuando alguno de los timers genera una interrupción por actualización (update interrupt),
 * que en este caso ocurre cuando el contador del timer ha llegado a su valor de autorecarga y vuelve a contar
 * desde 0 (overflow del contador). Esto último es válido cuando el contador cuenta de forma ascendente.
 * En este caso, el timer TIM2 genera una update interrupt cada 1 min, por lo que debemos incrementar el contador
 * de minutos cuando dicho timer interrumpe */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim->Instance == TIM2){
		contminutos++;
	}

}

/* Callback que se ejecuta cuando alguno de los canales de captura y comparación (capture/compare) de los
 * temporizadores genera una interrupción por output compare (comparación de salida). Esta interrupción se genera
 * cuando el valor del registro de capture/compare (CC) del canal correspondiente del timer iguala el valor del
 * contador.
 * En este caso, el CH1 del TIM2 genera una interrupción por OC cada 3 s, y cuando esto ocurre se procede a
 * activar la bandera para tomar muestras de viento y al mismo tiempo se deshabilitan las interrupciones externas.
 * Por otro lado, el CH1 del TIM3 genera una interrupción por OC cada 1 ms */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){

	uint16_t pulse; // Variable para leer el contenido del registro de capture/compare
	/* Debemos averiguar de qué temporizador provino la interrupción */
	if(htim->Instance == TIM2){
		HAL_NVIC_DisableIRQ(EXTI0_IRQn); // Deshabilitamos las interrupciones externas
		flag_viento=1; // Activamos la bandera
		pulse = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Leemos el registro de CC del CH1 del TIM2
		/* Preguntamos si el valor actual del registro de CC más el valor a sumar supera el valor de autorecarga
		 * del contador del TIM2. Si se cumple esto, cargamos en el registro el valor inicial (2999). De lo
		 * contrario, sumamos 3000 al contenido del registro */
		if(__HAL_TIM_GET_AUTORELOAD(htim)>=(pulse+3000)){
			/* Sumamos 3000 al registro de CC del CH1 del TIM2 */
			__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,(pulse+(3000)));
		}
		else {
			/* Cargamos el valor 2999 en el registro de CC */
			__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,2999);
		}

	}

	if(htim->Instance == TIM3){
		flag_muestras=1; // Activamos la variable bandera
		pulse = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Leemos el registro de CC del CH1 del TIM3
		/* Preguntamos si el valor actual del registro de CC más el valor a sumar supera el valor de autorecarga
		 * del contador del TIM3. Si se cumple esto, cargamos en el registro el valor inicial (59). De lo
		 * contrario, sumamos 60 al contenido del registro */
		if(__HAL_TIM_GET_AUTORELOAD(htim)>=(pulse+60)){
			/* Sumamos 60 al registro de CC del CH1 del TIM3 */
			__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,(pulse+(60)));
		}
		else {
			/* Cargamos el valor 59 en el registro de CC */
			__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,59);
		}
	}

}

/* Rutina que se ejecuta cuando el ADC finaliza la conversión. En este caso, la interrupción es generada por el
 * ADC1 cuando finaliza la conversión del canal 9 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){

	flag_conversion=1; // Activamos la bandera para indicar el fin de la conversión
}

/*¨Callback que se ejecuta cuando llega una interrupción externa. En este caso, se ejecuta cuando llega una
 * interrupción externa proveniente del pin A15, y cuando llega esta interrupción significa que se detectó un
 * pulso generado por el anemómetro */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	contpulsos++; // Incrementamos el contador de pulsos del anemómetro
}

/* Esta rutina adquiere los datos meteorológicos provenientes de los distintos sensores, para lo cual utiliza
 * funciones o rutinas de las bibliotecas de cada sensor. Los datos adquiridos se guardan en una variable
 * tipo estructura */
void Adquirir_Datos(METEO_DATA *data){

	AHT10_DATA tempyhum; // Variable para guardar los datos leídos de temperatura y humedad del AHT10
	float angprom; // �?ngulo promedio de la dirección del viento
	uint8_t contsigpos,contsigneg; // Variables para contar los signos positivos y negativos de los ángulos

	/* Leemos primero el dato actual de temperatura y actual del sensor AHT10. En este caso, tomamos una
	 * sola muestra de temperatura y humedad */
	tempyhum=AHT10_Read();
	data->temp=tempyhum.tcelsius; // Temperatura actual (en °C)
	data->humrel=tempyhum.hr; // Humedad relativa (en %)

	/* Ahora leemos el dato de presión atmosférica proveniente del sensor BMP180. Al igual que antes,
	 * tomamos una única muestra de presión atmosférica sin hacer ningún promedio */
	data->pres=BMP180_Get_TruePressure(BMP180_STANDARD); // Presión atmosférica (en hPa)

	/* Obtenemos ahora la precipitación acumulada mediante una lectura del pluviómetro */
	data->precip=Leer_Pluviometro(); // Precipitación acumulada (en mm)

	/* Obtenemos la dirección del viento a partir del promedio de las muestras de ángulo de la dirección del
	 * viento tomadas cada 3 s durante los últimos 5 minutos */
	angprom=Promedio_Angulo(dirvelviento.angdir); // Obtenemos el ángulo promedio de la dir. del viento
	contsigpos=Contar_Signos(dirvelviento.sigdir,positivo); // Contamos los signos positivos de los ángulos
	contsigneg=Contar_Signos(dirvelviento.sigdir,negativo); // Contamos los signos negativos de los ángulos
	// Hallamos la dir. del viento a partir del ángulo promedio y la cantidad de signos
	data->dirviento=Obtener_Dir_Viento(angprom,contsigpos,contsigneg);

	/* Obtenemos ahora la velocidad del viento promediando las muestras de velocidad de viento tomadas cada
	 * 3 s durante los últimos 5 minutos */
	data->velviento=Promedio_Muestras_Anemometro(dirvelviento.velv);
	cantmuestviento=0; // Reseteamos la variable que cuenta las muestras de viento

}

/* Rutina que envía los datos meteorológicos adquiridos al módulo Bluetooth mediante la interfaz UART, junto
 * con la fecha y hora actual */
void Enviar_Datos(METEO_DATA *data){

	/* Enviamos primero la fecha y hora actuales */
	Enviar_Fecha_Actual(&fechaactual,'F'); // Enviamos la fecha actual
	Enviar_Hora_Actual(&horaactual,'I'); // Enviamos la hora actual

	/* Enviamos ahora los datos meteorológicos adquiridos */
	Enviar_Dato_Float(data->temp,'T',2); // Temperatura (2 dígitos significativos)
	Enviar_Dato_Float(data->humrel,'H',2); // Humedad relativa (2 dígitos significativos)
	Enviar_Dato_Float(data->pres,'P',4); // Presión atmosférica (4 dígitos significativos)
	/* Si el viento no está calmo, enviamos la dirección del viento normalmente. En cambio, si el viento
	 * está calmo, enviamos un guión '-' en lugar de la dirección del viento ya que no tiene sentido
	 * indicar la dir. del viento cuando no hay viento */
	if(data->velviento>0){
		Enviar_Dir_Viento(data->dirviento); // Dirección del viento
		Enviar_Dato_Float(data->velviento,'V',2); // Velocidad del viento (2 dígitos significativos)
	}
	else {
		Enviar_Caracter("-", 'D'); // Enviamos un guión '-'
		Enviar_Dato_Float(data->velviento,'V',2); // Velocidad del viento (2 dígitos significativos)
	}
	Enviar_Dato_Float(data->precip,'L',3); // Precipitación acumulada (3 dígitos significativos)

}

/* Rutina que almacena los datos meteorológicos adquiridos en la memoria SD, junto con la fecha y hora
 * actuales. Para ello utiliza la interfaz de comunicación SPI */
void Almacenar_Datos(METEO_DATA *data){

	/* Escribimos primero la fecha actual en el archivo de texto de la memoria SD */
	Escribir_Fecha_Actual(&fechaactual);
	/* Ahora escribimos la hora actual en el archivo de texto */
	Escribir_Hora_Actual(&horaactual);
	/* Finalmente escribimos los datos meteorológicos adquiridos en el archivo de texto */
	Escribir_Dato_Float(data->temp,2," "); // Temperatura (2 dígitos significativos)
	Escribir_Dato_Float(data->humrel,2," "); // Humedad (2 dígitos significativos)
	Escribir_Dato_Float(data->pres,4," "); // Presión atmosférica (4 dígitos significativos)
	/* Si el viento no está calmo, guardamos la dirección del viento normalmente. En cambio, si el viento
	 * está calmo, guardamos un guión '-' en lugar de la dirección del viento ya que no tiene sentido
	 * indicar la dir. del viento cuando no hay viento */
	if(data->velviento>0){
		Escribir_Dir_Viento(data->dirviento," "); // Dirección del viento
		Escribir_Dato_Float(data->velviento,2," "); // Velocidad del viento (2 dígitos significativos)
	}
	else {
		Escribir_Caracter("-", " "); // Escribimos un guión para indicar que el viento es calmo
		Escribir_Dato_Float(data->velviento,2," "); // Velocidad del viento (2 dígitos significativos)
	}

	Escribir_Dato_Float(data->precip,3,"\n"); // Precipitación (3 dígitos significativos)
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
  /* Guardamos en los campos de la estructura SPISD los parámetros para la configuración de la tarjeta SD */
  spisd.FSM=Encendido;
  spisd.csPuerto = CS_GPIO_Port;
  spisd.csPin = CS_Pin;
  spisd.puertoSPI = &hspi2;
  spisd.sectorAddressing=1; //Asumimos SDHC (+2GB)
  spisd.hrtc = &hrtc;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  /* Obtenemos los coeficientes de calibración del sensor BMP180 para el cálculo de la presión */
  BMP180_Get_AllCalCoef();
  /* Función para calibrar el pluviómetro utilizando un peso conocido (en este caso, un peso de agua conocido) */
  HX711_Set_Scale(20);

  /* Realizamos el ajuste de la fecha y hora en el RTC, solicitándole al usuario que ingrese la fecha y hora actual */
  Ajustar_Hora_Actual();
  Ajustar_Fecha_Actual();

  // Las variables &USERFatFS y USERPath son creadas automáticamente por el File system
  f_mount(&USERFatFS,USERPath,0);

  /* Creamos un archivo de texto llamado "METEODATA" para poder guardar los datos meteorológicos */
  f_open(&USERFile,"Datos21.txt",FA_CREATE_ALWAYS | FA_WRITE);

  /* Limpiamos las interrupciones pendientes de los timers y el ADC */
  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC1);
  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC2);
  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
  /* Limpiamos las interrupciones externas pendientes */
  HAL_EXTI_ClearPending(&hexti, EXTI_TRIGGER_RISING);

  /* Inicializamos el timer TIM2, el cual generará la demora necesaria para la adquisición de datos */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);

  METEO_DATA datos; // Variable para guardar los datos meteorológicos leídos
  uint8_t buffer[4];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Tomar_Muestras_Viento(&dirvelviento); // Tomamos muestras de la dir. y vel. del viento cada 3 s
	  /* Si pasaron 5 minutos desde la última adquisición de datos, adquirimos datos nuevamente para luego
	   * enviarlos a la app y almacenarlos en la memoria */
	  if(contminutos==3){
		  contminutos=0; // Resetemaos el contador de minutos
		  Adquirir_Datos(&datos);
		  /* Obtenemos primero la fecha y hora actual para luego enviar y almacenar los datos */
	      fechaactual=Obtener_Fecha_Actual();
		  horaactual=Obtener_Hora_Actual();
		  Enviar_Datos(&datos);
		  Almacenar_Datos(&datos);
	  }
	  /* Preguntamos si se recibió la palabra 'TARA' a través del puerto serie. De ser así, procedemos a ajustar
	   * nuevamente la tara del sensor */
	  if(HAL_UART_Receive(&huart1, buffer, 4, 10) == HAL_OK){
		  /* Para saber si recibimos la palabra TARA, evaluamos cada uno de los valores en ASCII recibidos.
		   * - buffer[0]=84 (T)
		   * - buffer[1]=65 (A)
		   * - buffer[2]=82 (R)
		   * - buffer[3]=65 (A)
		   */
		  if(buffer[0]==84 && buffer[1]==65 && buffer[2]==82 && buffer[3]==65){
			  HX711_Set_Tare(20);
		  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x22;
  sTime.Minutes = 0x26;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  DateToUpdate.Month = RTC_MONTH_NOVEMBER;
  DateToUpdate.Date = 0x9;
  DateToUpdate.Year = 0x22;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  sConfigOC.Pulse = 2999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PD_SCK_GPIO_Port, PD_SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DOUT_Pin */
  GPIO_InitStruct.Pin = DOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD_SCK_Pin */
  GPIO_InitStruct.Pin = PD_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PD_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ANEMOMETRO_Pin */
  GPIO_InitStruct.Pin = ANEMOMETRO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ANEMOMETRO_GPIO_Port, &GPIO_InitStruct);

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
