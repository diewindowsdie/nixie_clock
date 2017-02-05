/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define BlinkingSecondsLED_Pin GPIO_PIN_0
#define BlinkingSecondsLED_GPIO_Port GPIOA
#define TimeSetModeLED_Pin GPIO_PIN_4
#define TimeSetModeLED_GPIO_Port GPIOA
#define EncoderPoleButton_Pin GPIO_PIN_5
#define EncoderPoleButton_GPIO_Port GPIOA
#define H_M_Switch_Pin GPIO_PIN_6
#define H_M_Switch_GPIO_Port GPIOA
#define EncoderB_Pin GPIO_PIN_7
#define EncoderB_GPIO_Port GPIOA
#define EncoderA_Pin GPIO_PIN_1
#define EncoderA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
//defines for the time unit that is currently selected to be changed
#define TIMEUNIT_MINUTES 0
#define TIMEUNIT_HOURS 1

//should we invert encoder events, i.e. decrease time when program thinks it it turned right. Useful for pin swapped encoders
#define ENCODER_DIRECTION_INVERTED 0

#define TOGGLE_TIME_UNIT_TO_CHANGE(timeUnit) (timeUnit ^= 1)

//minimum time between h/m switch button presses
#define H_M_BUTTON_DEBOUNCE_TIME 100

//minimum time between different "encoder turned" events
#define ENCODER_TURN_DEBOUNCE_TIME 30
/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
