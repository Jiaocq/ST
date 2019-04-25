void rt_hw_console_output(const char *str)
 {
 /* 进入临界段 */
 rt_enter_critical();

 /* 直到字符串结束 */
 while (*str!='\0') {
 /* 换行 */
 if (*str=='\n') {
 HAL_UART_Transmit( &huart1,(uint8_t *)'\r',1,1000);
 }
 HAL_UART_Transmit( &huart1,(uint8_t *)(str++),1,1000);
 }

 /* 退出临界段 */
 rt_exit_critical();
 }