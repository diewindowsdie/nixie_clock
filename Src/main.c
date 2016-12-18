/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "DS1307.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void Error_Handler(void);

static void MX_GPIO_Init(void);

static void MX_I2C1_Init(void);

static void MX_TIM14_Init(void);

static void MX_TIM16_Init(void);

static void MX_TIM17_Init(void);

static void MX_TIM3_Init(void);

static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/**
 * Increase current time for 500 ms and updates output
 */
static void updateOutput();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
volatile DS1307_Time currentTime = {.hours = 0x00, .minutes = 0x00, .seconds = 0x00, .halfSeconds = 0};

void updateOutput() {
    //increase currentTime
    if (currentTime.halfSeconds == 1) {
        currentTime.halfSeconds = 0;
        if (currentTime.seconds == 59) {
            currentTime.seconds = 0;
            if (currentTime.minutes == 59) {
                currentTime.minutes = 0;
                    if (currentTime.hours == 23) {
                        currentTime.hours = 0;
                    } else {
                        currentTime.hours++;
                    }
            } else {
                currentTime.minutes++;
            }
        } else {
            currentTime.seconds++;
        }
    } else {
        currentTime.halfSeconds = 1;
    }

    //todo actual output
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim14.Instance) {
        //Timer HTIM14 requests updated currentTime from DS1307
        //disable timer14 IRQ until read data from DS1307 is completed (to prevent two concurrent reads)
        HAL_NVIC_DisableIRQ(TIM14_IRQn);
        //disable set currentTime mode button to prevent concurrent read and write
        HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
        DS1307_RequestTimeReading();
    } else if (htim->Instance == htim16.Instance) {
        //Timer HTIM16 fires every 500 ms, blinks with seconds LED and increases seconds (if necessary, minutes and hours also)
        HAL_GPIO_TogglePin(BlinkingSecondsLED_GPIO_Port, BlinkingSecondsLED_Pin);
        updateOutput();
    } else if (htim->Instance == htim3.Instance) {
        //Timer HTIM3 resets currentTime set mode after 10 seconds
        HAL_GPIO_WritePin(TimeSetModeLED_GPIO_Port, TimeSetModeLED_Pin, GPIO_PIN_RESET);

        //todo disable encoder and h/m button interrupts
        HAL_NVIC_DisableIRQ(TIM3_IRQn);
    } else if (htim->Instance == htim17.Instance) {
        //Timer HTIM17 used to check if encoder button was pressed for long enough to toggle currentTime set mode
        //if timer actually fired, we no longer need it so disable
        HAL_NVIC_EnableIRQ(TIM17_IRQn);

        GPIO_PinState isTimeSetModeEnabled = HAL_GPIO_ReadPin(TimeSetModeLED_GPIO_Port, TimeSetModeLED_Pin);
        //button was hold for long enough, no need for other calls until this one is completed
        //disable currentTime set button (via IRQ) to prevent double writes on slow I2C bus
        HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
        HAL_GPIO_TogglePin(TimeSetModeLED_GPIO_Port, TimeSetModeLED_Pin);
        if (isTimeSetModeEnabled == GPIO_PIN_RESET) {
            //todo enable encoder and h/m button interrupts
            //clear timer count and prevent firing at once
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            __HAL_TIM_CLEAR_FLAG(&htim3, TIM_SR_UIF);

            //start timer that fires at button hold limit
            HAL_TIM_Base_Start_IT(&htim3);
            HAL_NVIC_EnableIRQ(TIM3_IRQn);

            HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
        } else {
            //todo disable encoder and h/m button interrupts

            //disable currentTime set mode reset timer
            HAL_NVIC_DisableIRQ(TIM3_IRQn);

            //disable timer IRQ until writing new currentTime and subsequent data reading from DS1307 is completed
            HAL_NVIC_DisableIRQ(TIM14_IRQn);

            //todo real new currentTime
            DS1307_Time newTime = {.hours = 23, .minutes = 45, .seconds = 25};
            DS1307_SetCurrentTime(&newTime);

            //we do not enable back IRQ here because we have to wait until writing data to DS1307 is completed
        }
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == hi2c1.Instance) {
        DS1307_Handle_Receive_Completed();
    }
    //start regular currentTime requesting and enable currentTime set button if bus is free and no operation is in progress
    //(no registers update were requested right after reading was completed)
    if (DS1307_GetCurrentState() == DS1307_READY) {
        //prevent interrupt at timer start
        __HAL_TIM_CLEAR_FLAG(&htim14, TIM_SR_UIF);
        HAL_TIM_Base_Start_IT(&htim14);
        HAL_NVIC_EnableIRQ(TIM14_IRQn);
        HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == hi2c1.Instance) {
        DS1307_Handle_Transmit_Completed();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == TimeSetButton_Pin) {
        //both rise and falling edge interrupt means we have to reset timer, so disable it anyway
        NVIC_DisableIRQ(TIM17_IRQn);
        GPIO_PinState timeSetButtonState = HAL_GPIO_ReadPin(TimeSetButton_GPIO_Port, TimeSetButton_Pin);
        if (timeSetButtonState == GPIO_PIN_RESET) {
            //high => low transition, button was pressed

            //clear timer count and prevent firing at once
            __HAL_TIM_SET_COUNTER(&htim17, 0);
            __HAL_TIM_CLEAR_FLAG(&htim17, TIM_SR_UIF);

            //start timer that fires at button hold limit
            HAL_TIM_Base_Start_IT(&htim17);
            NVIC_EnableIRQ(TIM17_IRQn);
        } else {
            //low => high transition, button was released
            //stop timer
            HAL_TIM_Base_Stop_IT(&htim17);
        }
    }
}

/* USER CODE END 0 */

int main(void) {

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM14_Init();
    MX_TIM16_Init();
    MX_TIM17_Init();
    MX_TIM3_Init();

    /* Initialize interrupts */
    MX_NVIC_Init();

    /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim16);

    //Timer14 shouldn't call interrupt routines until the DS1307 initialization and first data reading is completed
    NVIC_DisableIRQ(TIM14_IRQn);
    //SetTime button also shouldn't be available until first data reading from DS1307 is completed
    NVIC_DisableIRQ(EXTI4_15_IRQn);

    //initialize DS1307 and request first data from there
    DS1307_Initialize(&hi2c1, &Error_Handler);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

    }
    /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void) {

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }

    /**Configure the Systick interrupt time 
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**Configure the Systick 
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void) {
    /* I2C1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(I2C1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(I2C1_IRQn);
    /* TIM16_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(TIM16_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM16_IRQn);
    /* TIM14_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(TIM14_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM14_IRQn);
    /* TIM17_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(TIM17_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM17_IRQn);
    /* TIM3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    /* EXTI4_15_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* I2C1 init function */
static void MX_I2C1_Init(void) {

    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x4000096C;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }

    /**Configure Analogue filter 
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        Error_Handler();
    }

}

/* TIM3 init function */
static void MX_TIM3_Init(void) {

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 23999;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 4999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

}

/* TIM14 init function */
static void MX_TIM14_Init(void) {

    htim14.Instance = TIM14;
    htim14.Init.Prescaler = 11999;
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = 59999;
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
        Error_Handler();
    }

}

/* TIM16 init function */
static void MX_TIM16_Init(void) {

    htim16.Instance = TIM16;
    htim16.Init.Prescaler = 11999;
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = 499;
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.RepetitionCounter = 0;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
        Error_Handler();
    }

}

/* TIM17 init function */
static void MX_TIM17_Init(void) {

    htim17.Instance = TIM17;
    htim17.Init.Prescaler = 11999;
    htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim17.Init.Period = 999;
    htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim17.Init.RepetitionCounter = 0;
    htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
        Error_Handler();
    }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void) {

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, BlinkingSecondsLED_Pin | TimeSetModeLED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : BlinkingSecondsLED_Pin TimeSetModeLED_Pin */
    GPIO_InitStruct.Pin = BlinkingSecondsLED_Pin | TimeSetModeLED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : TimeSetButton_Pin */
    GPIO_InitStruct.Pin = TimeSetButton_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(TimeSetButton_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    while (1) {
    }
    /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
