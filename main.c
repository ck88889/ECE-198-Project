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
#include "liquidcrystal_i2c.h"
#include  "stdbool.h"
#include <string.h>
#include <stdio.h>
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HD44780_Init(2); //initialize LCD Display

  uint32_t mq_analog; ////MQ-7 sensor
  uint32_t mq_sum = 0; //MQ-7 sensor sum

  uint16_t sds_rx; //SDS-011 sensor
  uint16_t sds_sum = 0; //SDS-011 sensor sum
  uint8_t data[50]; //buffer for SDS-011 sensor

  //average modes
  bool mode = false; //alternate between concentration and over and under
  bool avg_on = false; //should show avg or not
  int count = 0; //countdown from
  const int STOP = 60; //number of intervals for average

  const int CO_MAX = 20; //in ppm
  const int PM_MAX = 12;
  //characters
  char snum[5];

  	/**
     * Converts carbon monoxide concentration into mili-grams per cubic meter of air
     *
     *@param con_ppm concentration in ppm (parts per million)
     *@return concentration converted into micro-grams per cubic meter of air
    */
    int carbon_conversion(int con_ppm){
        const double CARBON_MOL = 28.01;
        return ((CARBON_MOL * con_ppm * 1000) / 24.45)/1000;
    }

    /**
      * Checks if carbon monoxide is above or below EPA guideline valyes
      *
      *@param carbon monoxide concentration in ppm (parts per million)
      *@return whether it the CO concentration is over or under EPA guideline values
     */
     bool is_CO(int ppm_CO){
        if(ppm_CO < CO_MAX){
        	return true;
        }
        return false;
     }

     /**
      * Checks if particulate matter is above or below EPA guideline valyes
       *
       *@param particulate matter concentration in um/g^3
       *@return whether it the PM2.5 concentration is over or under EPA guideline values
     */
        bool is_PM25(int PM2_5){
          if(PM2_5  < PM_MAX){
            return true;
          }
            return false;
        }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //Getting input
	  mq_analog = carbon_conversion(HAL_ADC_GetValue(&hadc1)); //read analog data from MQ-7 sensor and convert measurements
	  HAL_UART_Receive_IT(&huart2, data, 9); //read SDS011 sensor into an array/buffer
	  sds_rx = (data[3]|data[2])/10; //calculate the amount of pm2.5 with formula: (high byte + low byte)/10

	  //when the blue button is pushed
	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)== GPIO_PIN_RESET){
	  	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //turn light on

	  	 if(avg_on){ //on -> off average mode
	  		 avg_on = false;
	  		 count = 0;
	  		 mq_sum = 0;
	  		 sds_sum = 0;
	  	 }else{ //off -> on
	  		 avg_on = true;
	  	 }
	  }else{
	  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //turn light off
	  }

	  HD44780_Clear();

	  //Carbon monoxide display
	  if(avg_on){
	  	 if(count == STOP){ //show avg carbon monoxide
	  	   itoa(mq_sum/STOP, snum, 10);
	  	 }else{
	  		itoa(mq_analog, snum, 10);
	  	 }
	  }else{ //show instanteous CO concentration
		  itoa(mq_analog, snum, 10); //turn analog numeric value into its corresponding character
	  }

	  HD44780_SetCursor(0,0);
	  HD44780_PrintStr("CO: ");
	  HD44780_PrintStr(snum);
	  HD44780_PrintStr(" mg/m3");

	  if(!mode){
	    HD44780_Clear();
	    HD44780_PrintStr("CO: ");

	    if(avg_on && count == STOP){
	    	if(is_CO(HAL_ADC_GetValue(&hadc1))){ //CO avg safe values
	    	 itoa(carbon_conversion(CO_MAX - mq_sum/STOP), snum, 10);
	    	 HD44780_PrintStr(snum);
	    	 HD44780_PrintStr(" BELOW");
	      }else{
	    	 itoa(carbon_conversion(mq_sum/STOP - CO_MAX), snum, 10);
	    	 HD44780_PrintStr(snum);
	    	 HD44780_PrintStr(" ABOVE");
	      }
	    }else if(is_CO(HAL_ADC_GetValue(&hadc1))){ //CO instanteous safe values
	    	itoa(carbon_conversion(CO_MAX - HAL_ADC_GetValue(&hadc1)), snum, 10);
	  	   HD44780_PrintStr(snum);
	  	   HD44780_PrintStr(" BELOW");
	    }else{
	  	   itoa(carbon_conversion(HAL_ADC_GetValue(&hadc1) - CO_MAX), snum, 10);
	  	   HD44780_PrintStr(snum);
	  	   HD44780_PrintStr(" ABOVE");
	  	}
	  }

	  if(avg_on){
		if(count != STOP){ //show countdown
		  count++;
		  itoa(count, snum, 10);
		  HD44780_SetCursor(14,0);
		  HD44780_PrintStr(snum);

		  mq_sum = mq_sum + mq_analog; //sum of CO
		  sds_sum = sds_sum + sds_rx; //sum of PM2.5
	  	}else{
	      HD44780_SetCursor(12,0); //signal to user that they're in avg mode
	  	  HD44780_PrintStr(" AVG");
	  	}
	 }

	  //Particulate matter display
	  if(avg_on){ //show avg PM2.5 concentration
	  	if(count == STOP){
	  		itoa(sds_sum/STOP, snum, 10);
	  	 }else{
	  		itoa(sds_rx, snum, 10);
	  	 }
	  }else{
		itoa(sds_rx, snum, 10);
	  }

	  HD44780_SetCursor(0,1);
	  if(mode){//displaying instanteous PM2.5
	  	  mode = false;
	  	  HD44780_PrintStr("PM2.5: ");
	  	  HD44780_PrintStr(snum);
	  	  HD44780_PrintStr(" ug/m3");
	  }else{
	  	HD44780_PrintStr("PM2.5: ");
	  	mode = true;
	  	if(avg_on && count == STOP){
	  		if(is_PM25(sds_sum)){ //PM25 instanteous avg safe values
	  		  itoa((sds_sum/STOP) - PM_MAX, snum, 10);
	  		  HD44780_PrintStr(snum);
	  	      HD44780_PrintStr(" ABOVE");
	        }else{
	  		  itoa(PM_MAX - (sds_sum/STOP), snum, 10);
	  		  HD44780_PrintStr(snum);
	  		  HD44780_PrintStr(" BELOW  ");
	  	    }
	  	}
	  	if(is_PM25(sds_rx)){ //PM25 instanteous safe values
	  	  itoa(PM_MAX - sds_rx, snum, 10);
	  	  HD44780_PrintStr(snum);
	  	  HD44780_PrintStr(" BELOW  ");
	  	}else{
	  	  itoa(sds_rx - PM_MAX, snum, 10);
	  	  HD44780_PrintStr(snum);
	  	  HD44780_PrintStr(" ABOVE");
	  	}
	  }

	  HAL_Delay(1000); //1 second
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  huart1.Init.BaudRate = 115200;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
