/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//_____________________________________________________________________________________________________________

//_____________________________________________________________________________________________________________
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
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//_____________________________________________________________________________________________________________________________ 
// Ð?nh nghia các pattern LED
typedef enum {
    PATTERN_REGULAR,       // LED sáng 200ms, t?t 800ms
    PATTERN_DOUBLE_BLINK,  // Nháy 2 l?n ng?n: 100ms on – 100ms off – 100ms on – 700ms off
    PATTERN_SOS,            // Nháy theo mã SOS
		PATTERN_TRIPLE_BLINK,
		PATTERN_DOUBLE_TWO_BLINK
} LedPattern_t;

/*
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
*/                                                                   //xxx
                                                                     //xxx
LedPattern_t currentPattern = PATTERN_TRIPLE_BLINK; 
                                                                     //xxx
/*                                                                   //xxx
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
*/


// --------------------------
// Pattern 1: Regular (200ms on, 800ms off)
#define REGULAR_ON 200
#define REGULAR_OFF 800
#define REGULAR_TOTAL (REGULAR_ON + REGULAR_OFF)

// --------------------------
// Pattern 2: Double Blink (100ms on, 100ms off, 100ms on, 700ms off)
#define DOUBLE_BLINK_ON1   100
#define DOUBLE_BLINK_OFF1  100
#define DOUBLE_BLINK_ON2   100
#define DOUBLE_BLINK_OFF2  700
#define DOUBLE_BLINK_TOTAL (DOUBLE_BLINK_ON1 + DOUBLE_BLINK_OFF1 + DOUBLE_BLINK_ON2 + DOUBLE_BLINK_OFF2)

// --------------------------
// Pattern 3: SOS pattern
// Gi? s?: Dot = 100ms on, 100ms off; Dash = 300ms on, 100ms off; Pause cu?i = 700ms
#define SOS_SHORT_ON  100
#define SOS_SHORT_OFF 100
#define SOS_LONG_ON   300
#define SOS_LONG_OFF  100
#define SOS_PAUSE     700
// S: dot dot dot, O: dash dash dash, S: dot dot dot


//---------------------------
//Pattern 4: triple blink
#define TRIPLE_BLINK_ON         100
#define TRIPLE_BLINK_OFF1       100
#define TRIPLE_BLINK_OFF2       100
#define TRIPLE_BLINK_OFF_FINAL  800
#define TRIPLE_BLINK_TOTAL (TRIPLE_BLINK_ON + TRIPLE_BLINK_OFF1 + TRIPLE_BLINK_ON + TRIPLE_BLINK_OFF2 + TRIPLE_BLINK_ON + TRIPLE_BLINK_OFF_FINAL)
//---------------------------
//Pattern5: double two blink
#define DBL2_ON         100
#define DBL2_OFF_SHORT  100
#define DBL2_PAUSE      200
#define DBL2_OFF_FINAL  800
#define PATTERN_DOUBLE_TWO_TOTAL  (DBL2_ON + DBL2_OFF_SHORT + DBL2_ON + DBL2_OFF_SHORT + DBL2_PAUSE + DBL2_ON + DBL2_OFF_SHORT + DBL2_ON + DBL2_OFF_FINAL)

uint32_t sosSequence[] = {
    SOS_SHORT_ON, SOS_SHORT_OFF, SOS_SHORT_ON, SOS_SHORT_OFF, SOS_SHORT_ON, SOS_SHORT_OFF, // S: dot dot dot
    SOS_LONG_ON, SOS_LONG_OFF, SOS_LONG_ON, SOS_LONG_OFF, SOS_LONG_ON, SOS_LONG_OFF,         // O: dash dash dash
    SOS_SHORT_ON, SOS_SHORT_OFF, SOS_SHORT_ON, SOS_SHORT_OFF, SOS_SHORT_ON, SOS_PAUSE          // S: dot dot dot + pause
};
#define SOS_SEQUENCE_LENGTH (sizeof(sosSequence) / sizeof(sosSequence[0]))

// --------------------------
// Bi?n dùng chung
static uint32_t ms_counter = 0;   // Ð?m s? ms (dùng cho pattern Regular & Double Blink)
static uint8_t sos_index = 0;     // Ch? s? c?a sequence SOS
static uint32_t sos_timer = 0;    // Ð?m th?i gian trong m?i bu?c c?a SOS

/**
  * @brief  Callback du?c g?i m?i khi TIM2 update (m?i 1ms)
  * @param  htim: con tr? d?n c?u trúc TIM_HandleTypeDef c?a TIM2
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        switch (currentPattern)
        {
            case PATTERN_REGULAR:
            {
                ms_counter++;
                if (ms_counter >= REGULAR_TOTAL)
                {
                    ms_counter = 0;
                }
                if (ms_counter < REGULAR_ON)
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                }
                else
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                }
            } break;
            
            case PATTERN_DOUBLE_BLINK:
            {
                ms_counter++;
                if (ms_counter >= DOUBLE_BLINK_TOTAL)
                {
                    ms_counter = 0;
                }
                if (ms_counter < DOUBLE_BLINK_ON1)
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                }
                else if (ms_counter < DOUBLE_BLINK_ON1 + DOUBLE_BLINK_OFF1)
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                }
                else if (ms_counter < DOUBLE_BLINK_ON1 + DOUBLE_BLINK_OFF1 + DOUBLE_BLINK_ON2)
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                }
                else
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                }
            } break;
            
            case PATTERN_SOS:
            {
                sos_timer++;
                if (sos_timer >= sosSequence[sos_index])
                {
                    sos_timer = 0;
                    sos_index++;
                    if (sos_index >= SOS_SEQUENCE_LENGTH)
                    {
                        sos_index = 0;
                    }
                }
                if ((sos_index % 2) == 0)
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                }
                else
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                }
            } break;
            
            case PATTERN_TRIPLE_BLINK:
            {
                ms_counter++;
                if (ms_counter >= TRIPLE_BLINK_TOTAL)
                {
                    ms_counter = 0;
                }
                if (ms_counter < TRIPLE_BLINK_ON)
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                }
                else if (ms_counter < TRIPLE_BLINK_ON + TRIPLE_BLINK_OFF1)
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                }
                else if (ms_counter < TRIPLE_BLINK_ON + TRIPLE_BLINK_OFF1 + TRIPLE_BLINK_ON)
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                }
                else if (ms_counter < TRIPLE_BLINK_ON + TRIPLE_BLINK_OFF1 + TRIPLE_BLINK_ON + TRIPLE_BLINK_OFF2)
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                }
                else if (ms_counter < TRIPLE_BLINK_ON + TRIPLE_BLINK_OFF1 + TRIPLE_BLINK_ON + TRIPLE_BLINK_OFF2 + TRIPLE_BLINK_ON)
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                }
                else
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                }
            } break;
            
            case PATTERN_DOUBLE_TWO_BLINK:
            {
                ms_counter++;
                if (ms_counter >= PATTERN_DOUBLE_TWO_TOTAL)
                {
                    ms_counter = 0;
                }
                // Chu?i 1:
                if (ms_counter < DBL2_ON)
                {
                    // L?n nháy 1: on
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                }
                else if (ms_counter < DBL2_ON + DBL2_OFF_SHORT)
                {
                    // off
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                }
                else if (ms_counter < DBL2_ON + DBL2_OFF_SHORT + DBL2_ON)
                {
                    // L?n nháy 2: on
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                }
                else if (ms_counter < DBL2_ON + DBL2_OFF_SHORT + DBL2_ON + DBL2_OFF_SHORT)
                {
                    // K?t thúc chu?i 1, off
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                }
                // Pause gi?a 2 chu?i nháy: 100ms off
                else if (ms_counter < DBL2_ON + DBL2_OFF_SHORT + DBL2_ON + DBL2_OFF_SHORT + DBL2_PAUSE)
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                }
                // Chu?i 2:
                else if (ms_counter < DBL2_ON + DBL2_OFF_SHORT + DBL2_ON + DBL2_OFF_SHORT + DBL2_PAUSE + DBL2_ON)
                {
                    // L?n nháy 1: on
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                }
                else if (ms_counter < DBL2_ON + DBL2_OFF_SHORT + DBL2_ON + DBL2_OFF_SHORT + DBL2_PAUSE + DBL2_ON + DBL2_OFF_SHORT)
                {
                    // off
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                }
                else if (ms_counter < DBL2_ON + DBL2_OFF_SHORT + DBL2_ON + DBL2_OFF_SHORT + DBL2_PAUSE + DBL2_ON + DBL2_OFF_SHORT + DBL2_ON)
                {
                    // L?n nháy 2: on
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                }
                else
                {
                    // Ngh? cu?i chu k?: off 700ms
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                }
            } break;
            
            default:
                break;
        }
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
