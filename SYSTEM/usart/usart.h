#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
	

#define BUFLEN 512      //���黺���С
typedef struct _UART_BUF
{
    char buf [BUFLEN+1];               
    unsigned int index ;
}UART_BUF;	
	
	
void uart_init(u32 bound);
void uart2_init(u32 bound);
void uart3_init(u32 bound);
void UART1_send_byte(char data);
void UART2_send_byte(char data);
void UART3_send_byte(char data);
void Uart1_SendStr(char*SendBuf);
void Uart2_SendStr(char*SendBuf);
void Uart3_SendStr(char*SendBuf);

void Clear_Buffer_UART1(void);


extern UART_BUF buf_uart1;     //PC
extern UART_BUF buf_uart2;     //4G
extern UART_BUF buf_uart3;     //TTL

#endif


