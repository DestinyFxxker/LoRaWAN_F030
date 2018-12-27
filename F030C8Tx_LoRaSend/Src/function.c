#include "function.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_uart.h"

extern UART_HandleTypeDef huart2;

void  DebugPrintf (char  *p_fmt, ...)
{
	if(1)
	{
			char    str[200u];
			uint8_t  len;

			va_list     vArgs;

			va_start(vArgs, p_fmt);

			vsprintf((char       *)str,
							 (char const *)p_fmt,
														 vArgs);

			va_end(vArgs);
			len = strlen(str);
//			UartPutBuffer(&Uart1, (uint8_t *)str, len);

			HAL_UART_Transmit(&huart2, (uint8_t *)str, len, 10);
	}
}
