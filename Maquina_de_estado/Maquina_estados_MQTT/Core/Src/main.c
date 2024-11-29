/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ESTADO_INICIO 0
#define ESTADO_CERRADO 2
#define ESTADO_CERRANDO 3
#define ESTADO_ABIERTO 4
#define ESTADO_ABRIENDO 5
#define ESTADO_ERR 6
#define ESTADO_T 1
#define ESTADO_F 0
#define RT_MAX 180
#define ERR_OK 0
#define ERR_LS 1
#define ERROR_RT 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int ESTADO_SIGUIENTE = ESTADO_INICIO;
int ESTADO_ACTUAL = ESTADO_INICIO;
int ESTADO_ANTERIOR = ESTADO_INICIO;

struct DATA_IO
{
	unsigned int LSC:1;
	unsigned int LSA:1;
	unsigned int SPP:1; // COMANDO PULSO PULSO
	unsigned int MA:1; // INDICA MOTOR ABIERTO
	unsigned int MC:1; // INDICA MOTOR CERRADO
	unsigned int Cont_RT; // CONTADOR DE RUN TIME EN SEGUNDOS
	unsigned int Led_A:1;
	unsigned int Led_C:1;
	unsigned int Led_ERR:1;
	unsigned int COD_ERR: 1;
	unsigned int  DATOS_READY:1;



}data_io;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int Fun_INICIO(void);
int Fun_CERRADO(void);
int Fun_CERRANDO(void);
int Fun_ABRIENDO(void);
int Fun_ABIERTO(void);
int Fun_ERR(void);
int main(
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	for(;;)
	{
		if(ESTADO_SIGUIENTE == ESTADO_INICIO)
		{
			ESTADO_SIGUIENTE == Fun_INICIO();
		}

		if(ESTADO_SIGUIENTE == ESTADO_CERRADO)
				{
					ESTADO_SIGUIENTE == Fun_CERRADO();
				}

		if(ESTADO_SIGUIENTE == ESTADO_CERRANDO)
				{
					ESTADO_SIGUIENTE == Fun_CERRANDO();
				}

		if(ESTADO_SIGUIENTE == ESTADO_ABRIERDO)
				{
					ESTADO_SIGUIENTE == Fun_ABRIENDO();
				}

		if(ESTADO_SIGUIENTE == ESTADO_ABIERTO)
				{
					ESTADO_SIGUIENTE == Fun_ABIERTO();
				}

		if(ESTADO_SIGUIENTE == ESTADO_ERR)
					{
						ESTADO_SIGUIENTE == Fun_ERR();
					}


	}

	int Fun_INICIO(void) {

	    // Código de la función
		ESTADO_ANTERIOR = ESTADO_INICIO;
		ESTADO_ACTUAL = ESTADO_INICIO;

		data_io.MC = FALSE;
		data_io.MA = FALSE;
		data_io.SPP = FALSE;

		data_io.Led_A = TRUE;
		data_io.Led_C = TRUE;
		data_io.Led_ERR =TRUE;
				DELAY();
		data_io.Led_A = FALSE;
		data_io.Led_C = FALSE;
		data_io.Led_ERR = FALSE;
		data_io.COD _ERR = FALSE;
		data_io.Cont_RT = 0;
		data_io.DATOS_READY = FALSE;

		//posiblemente necesitemos esperar que se lean.
		// podemos llamar la funcion que sea las entradas.
		//antes de que el codigo continue.

		while(!DATOS_READY) // ESPERAR ENTRADA DE DATOS

	for(;;)
	{
		if (data_io.LSC == TRUE && data_io.LSA == FALSE)
		{
			return ESTADO_CERRADO;
		}

		if (data_io.LSC == TRUE && data_io.LSA == TRUE)
				{
					return ESTADO_ERR;
				}

		if (data_io.LSC == FALSE && data_io.LSA == FALSE)
				{
					return ESTADO_CERRANDO;
				}

		if (data_io.LSC == FALSE && data_io.LSA == TRUE)
				{
					return ESTADO_CERRANDO;
				}


	}
	};


	int Fun_CERRADO(void) {

	    // Código de la función
		        ESTADO_ANTERIOR = ESTADO_ACTUAL;
				ESTADO_ACTUAL = ESTADO_CERRADO;
				data_io.MC = FALSE;
				data_io.SPP = FALSE;
				data_io.Led_A = FALSE;
				data_io.Led_C = FALSE;
				data_io.Led_ERR = FALSE;



		    // Código de la función
		for(;;)
		{
			if (data_io.SPP == TRUE)
			{
				data_io.SPP == FALSE;
				return  ESTADO_ABRIENDO;
			}
		}

		};

	int Fun_CERRANDO(void) {

		    // Código de la función
		ESTADO_ANTERIOR = ESTADO_ACTUAL;
				ESTADO_ACTUAL = ESTADO_CERRANDO;
				data_io.MC = TRUE;
				data_io.Cont_RT = 0;


				data_io.Led_A = FALSE;
				data_io.Led_C = TRUE;
				data_io.Led_ERR = FALSE;
		for(;;)
		{
			if (data_io.LSC == TRUE)
			{
				return ESTADO_CERRANDO;
			}
			if (data_io.Cont_RT > RT_MAX)
						{
							return ESTADO_ERR;
						}
		}
		};

	int Fun_ABRIENDO(void) {
		 ESTADO_ANTERIOR = ESTADO_ACTUAL;
		    ESTADO_ACTUAL = ESTADO_ABRIENDO;
		   data
		    data_io.Cont_RT = 0;
		    data_io.led_C = FALSE;
		    data_io.led_A = TRUE;
		    data_io.led_ER = FALSE;
		    data.
		    // Código de la función
		for(;;)
		{
			 if (data_io.LSA == TRUE) // Si el sensor de apertura está activo//abrio
			        {
			            return ESTADO_ABIERTO;
			        }
			        if (data_io.Cont_RT > RT_MAX) // Si se supera el tiempo máximo ¡¡error
			        {
			            return ESTADO_ERROR;
			        }

		}
		};

	int Fun_ABIERTO(void) {
		 ESTADO_ANTERIOR = ESTADO_ACTUAL;
		    ESTADO_ACTUAL = ESTADO_ABIERTO;
		    data_io.MA = FALSE;
		    data_io.led_C = FALSE;
		    data_io.led_A = FALSE;
		    data_io.led_ER = FALSE;
		    // Código de la función
		for(;;)
		{
		    if (data_io.SPP == TRUE)
		        {
		            data_io.SPP = FALSE;
		            return ESTADO_CERRANDO;
		        }
		}
		};


	int Fun_ERR(void) {
		   ESTADO_ANTERIOR = ESTADO_ACTUAL;
		    ESTADO_ACTUAL = ESTADO_ERROR;
		    data_io.MA = FALSE;
		    data_io.MC = FALSE;
		    data_io.led_C = FALSE;
		    data_io.led_A = FALSE;
		    data_io.led_ER = TRUE; // LED DE ERROR ON
		    data_io.COD_ERR = ERROR_RT; // ERROR EXCEDIDO

		    // Código de la función
		for(;;)
		{
			 if (data_io.LSC == FALSE && data_io.LSA == FALSE) // ERR
			        {
			            if (data_io.RSBT == FALSE)
			            {
			                return ESTADO_ACTUAL;
			            }

			            // SI SE PRESIONA RESET
			            data_io.RSBT = TRUE;
			            return ESTADO_INIT;
			        }
		}
		};

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
