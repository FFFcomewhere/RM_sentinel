#ifndef __UART7_H
#define __UART7_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "stm32f4xx.h"

#define EN_UART7_RX 			1		//使能（1）/禁止（0）串口1接收
	
//#define JUDGE_BUF_LEN1   	200    //定义了裁判系统数据接收长度

#define VISION_BUFFER_LEN 200

void uart7_init(u32 bound);

typedef __packed struct 
{
	uint16_t VERSION_BUFFER;

} vision_buffer_t;

void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);

void Usart_SendString( USART_TypeDef * pUSARTx, char *str);

void Uart7_SendChar(uint8_t Data);

#endif
