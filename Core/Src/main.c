
#include "main.h"

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define TRIG_PIN GPIO_PIN_11
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_8
#define ECHO_PORT GPIOA

// Global Variables Needed

uint8_t rxData;
uint8_t i =0;
uint16_t angle =2500;
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;

uint16_t counterh=0;
uint16_t counterm=0;
uint16_t counterl=0;

uint32_t pMillis;
uint32_t Intial_time = 0;
uint32_t Final_time = 0;
uint16_t Distance  = 0;  // cm
uint16_t Capacity  = 0;

uint8_t data_Ultrasonic_Red[] = "*LR255G0B0*";
uint8_t data_Ultrasonic_Yellow[] = "*LR255G255B0*";
uint8_t data_Ultrasonic_Green[] = "*LR0G255B0*";
uint8_t data_IR_Green[] = "*VR0G255B0*";
uint8_t data_IR_Red[] = "*VR255G0B0*";

uint8_t dataHigh[] = "*KHigh*";
uint8_t dataMedium[] = "*KMedium*";
uint8_t dataLow[] = "*KLow*";
uint8_t datafull[] = "*TFull*";
uint8_t dataempty[] = "*TEmpty*";


uint8_t Servo_command = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);	//for Clock configuration
static void MX_GPIO_Init(void);	//GPIO initialization
static void MX_TIM1_Init(void);		//timer 1 initialization
static void MX_TIM2_Init(void);		//timer2 initialization
static void MX_USART1_UART_Init(void);		//usart1 initialization
/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);							   // start timer 1 for
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);                // start timer 2 for PWM channel
  /* USER CODE END 2 */


  while (1)
  {
	  /*
	     Checking if action happened or NOT
	     if Servo_command is set, so the pet or user pressed a button
	     otherwise the Servo_command is cleared
	  */
	  if(Servo_command==1)
	  {
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 7500); //Open Servo Motor
			HAL_Delay(600);  									//Delay until food drops
			for(angle=7500; angle>=2500; angle -=50)			//Close Servo Motor wisely
			{
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, angle);
				HAL_Delay(1);
			}

			for (i=0;i<6;i++) 									//Toggling LED & Buzzer
			{
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);		//pin A0 is toggled
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);		//pin C13 is toggled
				HAL_Delay(80);
			}
			Servo_command = 0;	//Servo off, turn off after being on
	  }

	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 2500); // return servo to original location

	  // IR Sensor Part
	   /*
	   * If the sensor is reading 0, food exists
	   * otherwise, dish status is indicating low food quantity
	   */
	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0)	//if The level is High in the dish
	  {
		  HAL_UART_Transmit(&huart1, data_IR_Green, sizeof(data_IR_Green), 10);		//Green led on app
	  }
	  else
	  {
		  HAL_UART_Transmit(&huart1, data_IR_Red, sizeof(data_IR_Red), 10);		//Red is on when Low level
	  }

	  // Ultra-sonic Sensor Part
	  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	  __HAL_TIM_SET_COUNTER(&htim1, 0);
	  while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
	  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	  pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
	  // Wait for the echo pin to go high
	  while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
	  Intial_time = __HAL_TIM_GET_COUNTER (&htim1);

      pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
	  // Wait for the echo pin to go low
      while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
      Final_time = __HAL_TIM_GET_COUNTER (&htim1);

      // Measuring Distance
      Distance = (Final_time-Intial_time)* 0.034/2;  //velocity of sound 0.034 cm/us

      // Cases of different distances

      // Counters created for preventing 1 or 2 error values from the ultrasonic sensor
     if(Distance >0 && Distance <= 10)		//if distance is between 0 and 10
        {
    	    counterh++;
    	    counterm=0;
    	    counterl=0;
    	    if(counterh >= 3)
    	    {
    	    	HAL_UART_Transmit(&huart1, data_Ultrasonic_Green, sizeof(data_Ultrasonic_Green), 10);
    	    	HAL_UART_Transmit(&huart1, dataHigh, sizeof(dataHigh), 10);
    	    }
		}
      else if (Distance > 10 && Distance <= 18)
		{
    	    counterh=0;
			counterm++;
			counterl=0;
			if (counterm >= 3)
			{
				HAL_UART_Transmit(&huart1, data_Ultrasonic_Yellow, sizeof(data_Ultrasonic_Yellow), 10);
				HAL_UART_Transmit(&huart1, dataMedium, sizeof(dataMedium), 10);
			}
		}
      else
      	{
    	   counterh=0;
    	   counterm=0;
    	   counterl++;
     	   if(counterl >= 3)
     	   {
     		   HAL_UART_Transmit(&huart1, data_Ultrasonic_Red, sizeof(data_Ultrasonic_Red), 10);
     		   HAL_UART_Transmit(&huart1, dataLow, sizeof(dataLow), 10);
     	   }
		}
	 HAL_UART_Receive_IT(&huart1,&rxData,1); // Enabling interrupt receive
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  htim1.Init.Prescaler = 71;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Test_LED_GPIO_Port, Test_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Led___Buzzer_Pin|Ultrasonic_Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Test_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Test_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Led__Pin Ultrasonic_Trig_Pin */
  GPIO_InitStruct.Pin = Led___Buzzer_Pin|Ultrasonic_Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_sensor_Pin */
  GPIO_InitStruct.Pin = IR_sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_sensor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Push_Button_Pin */
  GPIO_InitStruct.Pin = Push_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Push_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ultrasonic_Echo_Pin */
  GPIO_InitStruct.Pin = Ultrasonic_Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Ultrasonic_Echo_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

// Push Button Interrupt

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	currentMillis = HAL_GetTick();
	  if (GPIO_Pin == GPIO_PIN_1  								// button is pressed
			  && (currentMillis - previousMillis > 10)  		// Debouncing Effect condition
			  && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 1)		// IR is sensing No food
	  {
	    Servo_command = 1;
	    previousMillis = currentMillis;
	  }
}

// Mobile Application Interrupt

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART1)
  {
    if(rxData== 'Y') // User pressed the (Add Food) button
    {
    	Servo_command = 1;
    }
    HAL_UART_Receive_IT(&huart1,&rxData,1); // Enabling interrupt receive again
  }
}


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

void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif
