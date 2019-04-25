/*
 * File      : led.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2013, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-11-15     bright       the first version
 */

#include <rtthread.h>
#include "led.h"
// extern long list_thread(void);
//extern void list_mem( ); 
//extern rt_int16_t cpu_usage_get(void);

/*
LED_GREEN: PC13
*/
/* led thread entry */
static void led_thread_entry(void* parameter)
{
//	extern long list_thread(void);
//	while(1)
	{
//        rt_hw_led_on();
////		us_kprintf("rt_hw_led_on\r\n");
        rt_thread_delay(RT_TICK_PER_SECOND);

        rt_hw_led_off();
//				us_kprintf("rt_hw_led_off\r\n");
//cpu_usage_get();
//		list_thread();
        rt_thread_delay(RT_TICK_PER_SECOND);
	}
}
/* Initial led gpio pin  */
int rt_hw_led_init(void)
{
 	rt_thread_t led_thread;


	    GPIO_InitTypeDef GPIO_Initure;
    GPIO_LED_CLOCK_ENABLE();
     GPIO_Initure.Pin = LED_PIN;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LED_PORT, &GPIO_Initure);
     
 
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
	    /* Create led thread */
    led_thread = rt_thread_create("led",
    		led_thread_entry, RT_NULL,
    		512, 22, 20);
    if(led_thread != RT_NULL)
    	rt_thread_startup(led_thread);
    return 0;
}
/* Initial components for device */
INIT_APP_EXPORT(rt_hw_led_init);
