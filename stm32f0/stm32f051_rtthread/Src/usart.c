void rt_hw_console_output(const char *str)
 {
 /* �����ٽ�� */
 rt_enter_critical();

 /* ֱ���ַ������� */
 while (*str!='\0') {
 /* ���� */
 if (*str=='\n') {
 HAL_UART_Transmit( &huart1,(uint8_t *)'\r',1,1000);
 }
 HAL_UART_Transmit( &huart1,(uint8_t *)(str++),1,1000);
 }

 /* �˳��ٽ�� */
 rt_exit_critical();
 }