 

#ifndef __LED_H__
#define __LED_H__

#include "led.h"

 

#define LED1_PIN GPIO_PIN_2
#define LED1_PORT GPIOD
#define LED1ON      HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET)//GPIO_SetBits(GPIOC, GPIO_Pin_13)
#define LED1OFF     HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET)
#define LED1TOG		HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN)

#define LED2_PIN GPIO_PIN_3
#define LED2_PORT GPIOD
#define LED2ON      HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET)//GPIO_SetBits(GPIOC, GPIO_Pin_13)
#define LED2OFF     HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET)
#define LED2TOG		HAL_GPIO_TogglePin(LED2_PORT, LED2_PIN)
#endif

