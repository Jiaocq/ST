 

#ifndef __LED_H__
#define __LED_H__

#include <stm32h7xx_hal.h>

#define LED_PIN GPIO_PIN_0
#define LED_PORT GPIOB
#define rt_hw_led_off()       HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)//GPIO_SetBits(GPIOC, GPIO_Pin_13)
#define rt_hw_led_on()      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#define rt_hw_led_tog() 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN)

#define LED1_PIN GPIO_PIN_14
#define LED1_PORT GPIOB
#define LED1ON      HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET)//GPIO_SetBits(GPIOC, GPIO_Pin_13)
#define LED1OFF     HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET)
#define LED1TOG		HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN)

#define LED2_PIN GPIO_PIN_7
#define LED2_PORT GPIOB
#define LED2ON      HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET)//GPIO_SetBits(GPIOC, GPIO_Pin_13)
#define LED2OFF     HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET)
#define LED2TOG		HAL_GPIO_TogglePin(LED2_PORT, LED2_PIN)
#endif

