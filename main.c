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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void delay(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);
	while(__HAL_TIM_GET_COUNTER(&htim1)<time);
}


// Let's write the callback function
uint16_t target = 0;
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint16_t Distance  = 0;
uint16_t targetD = 50;
uint16_t MAX_DISTANCE=100;
uint16_t action;
#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA
 //interrupt by sensor ultra sonic
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  //if the interrupt source is channel1
	{
		if (Is_First_Captured==0) 					//if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  				//set the first captured as true

			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   			//if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  			//read second value


			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;

		    if(Distance > MAX_DISTANCE)
		    {
		    	Distance = MAX_DISTANCE;
		    }
		    if(Distance == 0)
		    {
		    	Distance = MAX_DISTANCE;
		    }

			if(Distance<targetD && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == SET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == SET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == SET )
			{
				target=1;
			}
			else
			{
				target=0;
			}

			Is_First_Captured = 0; 					//set it back to false
			__HAL_TIM_SET_COUNTER(htim, 0);			//reset the counter


			//set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);

		}
	}
}

//interrupted by sensor TCR5000
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == RESET  && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == SET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == SET)
	{
		stop();
		HAL_Delay(10);
		backward(400);
		HAL_Delay(100);
		stop();
		HAL_Delay(10);
		turnright(450);
		action = 1;
		/*-----left sensor --> backward and turn right-----*/
	}
	else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)== SET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == RESET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)== SET)
	  {
		stop();
		HAL_Delay(10);
		backward(400) ;
		HAL_Delay(100);
		stop();
		HAL_Delay(10);
		turnleft(450);
		action = 0;
		/*-----right sensor --> backward and turn left-----*/
	  }
	else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)== SET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == SET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)== RESET)
	{
		stop();
		HAL_Delay(10);
		forward(500);
	    HAL_Delay(75);
	    /*-----back sensor --> forward-----*/
	}
	else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)== RESET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)== RESET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)== SET)
	{
		stop();
		HAL_Delay(10);
		backward(500) ;
		HAL_Delay(500);
		 /*-----left sensor, right sensor --> backward-----*/
	}
	else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)== RESET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)== SET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)== RESET)
	{
		stop();
		HAL_Delay(10);
		turnright(600) ;
		HAL_Delay(350) ;
		forward(450)   ;
		HAL_Delay(250) ;
		action = 1       ;

		  /*-----left sensor, back sensor --> turn right, forward-----*/
	}
	else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)== SET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == RESET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)== RESET)
	{
		stop();
		HAL_Delay(10);
		turnleft(600)  ;
		HAL_Delay(350) ;
		forward(450)   ;
		HAL_Delay(250) ;
		action = 0       ;
		   /*-----right sensor, back sensor --> turn left, forward-----*/
	}


}


// Let's write the callback function

// READ ULTRA SONIC
void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);     // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}

//moving function
void turnright(int tempSpeed)
{
       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET); //in1
       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET); //in2
       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, RESET); //in3
       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, SET)  ; //in4
       htim2.Instance->CCR1 = tempSpeed           ; //PA15
       htim2.Instance->CCR2 = 0                   ; //PB3
}

void turnleft(int tempSpeed)
{

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET); //in1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET)  ; //in2
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, RESET); //in3
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, RESET); //in4
	htim2.Instance->CCR2 = tempSpeed           ; //PB3
	htim2.Instance->CCR1 = 0                   ; //PA15
}

void backward(int tempSpeed)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET)      ; //in1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET)    ; //in2
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, SET)      ; //in3
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, RESET)    ; //in4
	htim2.Instance->CCR1 = tempSpeed               ; //PA15
	htim2.Instance->CCR2 = tempSpeed               ; //PB3
}

void forward(int tempSpeed)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET)     ; //in1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET)       ; //in2
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, RESET)     ; //in3
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, SET)       ; //in4
	htim2.Instance->CCR1=tempSpeed                  ; //PA15
	htim2.Instance->CCR2=tempSpeed                  ; //PB3
}

void stop()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET); //in1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET); //in2
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, RESET); //in3
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, RESET); //in4
}

void turnright_2_wheel(int tempSpeed)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET)     ; //in1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET)       ; //in2
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, SET)       ; //in3
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, RESET)     ; //in4
	htim2.Instance->CCR1=tempSpeed                  ; //PA15
	htim2.Instance->CCR2=tempSpeed					; //PB3
}

void turnleft_2_wheel(int tempSpeed)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET)     	; //in1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET)     ; //in2
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, RESET)     ; //in3
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, SET)     	; //in4
	htim2.Instance->CCR1=tempSpeed                  ; //PA15
	htim2.Instance->CCR2=tempSpeed					; //PB3
}

void attack()
{
	if((target == 1) && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == SET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == SET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == SET)
	{
		forward(750);
	}
	else if (!(target == 1) && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == SET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == SET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == SET)
	{
		 if(action == 1)
		 {
			 turnright(500);
		 }
		 else
		 {
			 turnleft(500);
		 }
	}


}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);
  //turnright(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    HCSR04_Read();
      HAL_Delay(200);
      attack();


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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 7999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
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

  /* USER CODE END TIM3_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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


