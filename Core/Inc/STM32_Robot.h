#ifndef STM32_ROBOT_H
#define STM32_ROBOT_H

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define LED 		GPIO_PIN_5		//PA5
#define BUZZER 		GPIO_PIN_6		//PB6
#define BUTTON 		GPIO_PIN_10 	//PA10

#define FOWARD 		GPIO_PIN_RESET
#define BACKWARD 	GPIO_PIN_SET

#define PWM_MAX		39321

#define NUMBER_SNR	4
#define DIGIAL_SNR	0
#define ANALOG_SNR	1


typedef struct Engine{
	TIM_HandleTypeDef* tim;
	uint32_t timChl;
	GPIO_TypeDef* dirPinPort;
	uint32_t dirPin;
	int8_t pwmVal;
}Engine;

typedef struct Sensor{
	char snrName[10];
	uint8_t snrType;
	GPIO_TypeDef* snrPinPort;
	uint32_t snrPin;
	uint32_t snrVal;
	ADC_HandleTypeDef* snrAdc;
	ADC_ChannelConfTypeDef snrAdcChannel;
}Sensor;


typedef struct Robot{
	Engine engLeft;
	Engine engRight;
	UART_HandleTypeDef* uartUSB;
	UART_HandleTypeDef* uartBlth;
	UART_HandleTypeDef* uartSelected;
	GPIO_TypeDef* blthPinPort;
	uint32_t blthPin;
	uint8_t blthState;

	Sensor snr[NUMBER_SNR];

	uint8_t StartProcedureFlag;
}Robot;

void robotDefaultValuesInit(Robot* rob, TIM_HandleTypeDef* timEngLeft, TIM_HandleTypeDef* timEngRight, UART_HandleTypeDef* uartUSB, UART_HandleTypeDef* uartBlth, UART_HandleTypeDef* uartSelected, ADC_HandleTypeDef* adc);
void robotSetEnginePwm(Robot* rob, int8_t pwmValLeft, int8_t pwmValRight);
void robotTelemetry(Robot* rob);
void robotManualSteering(Robot* rob);
void robotReadSensors(Robot* rob);
void robotMove(Robot* rob);
void robotStartProcedure(Robot* rob);
void robotWallBouncer(Robot* rob);
void robotLightFollower(Robot* rob);
void robotLineFollowerV1(Robot* rob);
#endif //STM32_ROBOT_H
