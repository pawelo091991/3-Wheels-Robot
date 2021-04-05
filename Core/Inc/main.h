/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define Left_ADC_Light_Sensor_Pin GPIO_PIN_0
#define Left_ADC_Light_Sensor_GPIO_Port GPIOC
#define Left_GND_Light_Sensor_Pin GPIO_PIN_1
#define Left_GND_Light_Sensor_GPIO_Port GPIOC
#define Right_Contact_Sensor_Pin GPIO_PIN_0
#define Right_Contact_Sensor_GPIO_Port GPIOA
#define Left_Contact_Sensor_Pin GPIO_PIN_1
#define Left_Contact_Sensor_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Right_ADC_Light_Sensor_Pin GPIO_PIN_5
#define Right_ADC_Light_Sensor_GPIO_Port GPIOC
#define Left_5V_Light_Sensor_Pin GPIO_PIN_0
#define Left_5V_Light_Sensor_GPIO_Port GPIOB
#define Right_GND_Light_Sensor_Pin GPIO_PIN_6
#define Right_GND_Light_Sensor_GPIO_Port GPIOC
#define Right_Engine_Directiob_Pin GPIO_PIN_7
#define Right_Engine_Directiob_GPIO_Port GPIOC
#define Right_5V_Light_Sensor_Pin GPIO_PIN_8
#define Right_5V_Light_Sensor_GPIO_Port GPIOC
#define USART1_TX_Bluetooth_Pin GPIO_PIN_9
#define USART1_TX_Bluetooth_GPIO_Port GPIOA
#define USART1_RX_Bluetooth_Pin GPIO_PIN_10
#define USART1_RX_Bluetooth_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Bluetooth_Power_Supply_Pin GPIO_PIN_15
#define Bluetooth_Power_Supply_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Left_Engine_Direction_Pin GPIO_PIN_5
#define Left_Engine_Direction_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_6
#define Buzzer_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
