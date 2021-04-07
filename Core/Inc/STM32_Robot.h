/*
 * File: STM32_Robot.h
 * Author: 	Paweł Sporna (pawel.sporna@gmail.com)
 * Date: 	07.12.2020
 *
 * Summary of File:
 *
 * 	This file contains functions for 3 Wheels Robot with two electric engines and sets of sensors.
 * 	The code is written in object style using structs for defining engines, sensors and robot itself
 * 	Currently 4 programs are available: manual steering, wall bouncing with tact switches,
 * 	light follower and line follower with LED diodes and photoresistors
 */

#ifndef STM32_ROBOT_H
#define STM32_ROBOT_H

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#define LED_PIN 		GPIO_PIN_5		//PA5
#define LED_PORT		GPIOA
#define BUZZER_PIN 		GPIO_PIN_6		//PB6
#define BUZZER_PORT		GPIOB
#define BUTTON_PIN 		GPIO_PIN_10 	//PA10
#define BUTTON_PORT		GPIOA

#define FOWARD 			GPIO_PIN_RESET
#define BACKWARD 		GPIO_PIN_SET

#define PWM_MAX			39321
#define START_CNT_NUM	20000

#define NUMBER_SNR		4
#define DIGIAL_SNR		0
#define ANALOG_SNR		1


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

	uint8_t StartProcedureFinished;
	uint32_t StartProcedureCountNum;
}Robot;


/*
 * void robotDefaultValuesInit(Robot* rob, TIM_HandleTypeDef* timEngLeft, TIM_HandleTypeDef* timEngRight, UART_HandleTypeDef* uartUSB, UART_HandleTypeDef* uartBlth, UART_HandleTypeDef* uartSelected, ADC_HandleTypeDef* adc);
 *
 * Summary of the robotDefaultValuesInit function:
 * 	This function initiate robot engines, sensors and comunication based on stucture pointers provided in parameters
 *
 * Parameters: 	rob: a pointer to object with LCD and I2C configuration data
 * 				timEngLeft: pointer to PWM stucture to be used for left enigne control
 * 				timEngRight: pointer to PWM structure to be used for right engine control
 * 				uartUSB: pointer to UART structure to be used for communication between robot and terminal via USB cabel
 * 				uartBlth: pointer to UART structure to be used for communication between robot and terminal via Bluetooth HC-06
 *				uartSelected: pointer to UART structure to select if uartUSB or uartBlth will be selected for communication
 *				adc: pointer to ADC structure to be used for analog sensors
 *
 * Return Value : None
 *
 * Description:
 * 	This function assignes peripherials(ADC, timers and UART) to Robot sturcture pointed by rob
 */
void robotDefaultValuesInit(Robot* rob, TIM_HandleTypeDef* timEngLeft, TIM_HandleTypeDef* timEngRight, UART_HandleTypeDef* uartUSB, UART_HandleTypeDef* uartBlth, UART_HandleTypeDef* uartSelected, ADC_HandleTypeDef* adc);


/*
 * robotSetEnginePwm(Robot* rob, int8_t pwmValLeft, int8_t pwmValRight)
 *
 * Summary of the robotSetEnginePwm function:
 * 	This function initiate robot engines, sensors and comunication based on stucture pointers provided in parameters
 *
 * Parameters: 	rob: a pointer to object with LCD and I2C configuration data
 * 				timEngLeft: pointer to PWM stucture to be used for left enigne control
 * 				timEngRight: pointer to PWM structure to be used for right engine control
 * 				uartUSB: pointer to UART structure to be used for communication between robot and terminal via USB cabel
 * 				uartBlth: pointer to UART structure to be used for communication between robot and terminal via Bluetooth HC-06
 *				uartSelected: pointer to UART structure to select if uartUSB or uartBlth will be selected for communication
 *				adc: pointer to ADC structure to be used for analog sensors
 *
 * Return Value : None
 *
 * Description:
 * 	This function assignes peripherials(ADC, timers and UART) to Robot sturcture pointed by rob
 */
void robotSetEnginePwm(Robot* rob, int8_t pwmValLeft, int8_t pwmValRight);


/*
 * void robotTelemetry(Robot* rob)
 *
 * Summary of the robotTelemetry function:
 * 	This function serializes all robot parameters into JSON and send it via UART
 *
 * Parameters: 	rob: a pointer to object with LCD and I2C configuration data
 *
 * Return Value : None
 *
 * Description:
 * 	This function serializes engines, sensors, and program statuses into JSON and send it via UART
 */
void robotTelemetry(Robot* rob);
void robotManualSteering(Robot* rob);
void robotReadSensors(Robot* rob);
void robotMove(Robot* rob);

/*
 * void robotStartProcedure(Robot* rob)
 *
 * Summary of the robotStartProcedure function:
 * 	This function engage start procedure in robot to ensure that robot will start when user want it to
 *
 * Parameters: 	rob: a pointer to object with LCD and I2C configuration data

 *
 * Return Value : None
 *
 * Description:
 * 	This function run infinite while loop until user will switch on both contact switches for defined time
 * 	When both switches are on, LED is switched on to show that procedure is counting to defined period time
 * 	If user will switched off one of switch, procedure start from the beginning
 * 	When start procedure is completed, it buzzes for 0.5 second and function exits
 */
void robotStartProcedure(Robot* rob);
void robotWallBouncer(Robot* rob);
void robotLightFollower(Robot* rob);
void robotLineFollowerV1(Robot* rob);
void robotSwitchBuzzer(Robot* rob, uint8_t state);
void robotSwitchLED(Robot* rob, uint8_t state);
#endif //STM32_ROBOT_H
