#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
	

#define BUFLEN 512      //数组缓存大小
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

extern uint8_t UART3_RX_BUF[];  //
extern uint8_t UART3_RX_Index;

extern double value_xishu;
extern double value_yuliang;
extern uint16_t value_shiji;
extern uint16_t last_value;
extern uint32_t value_yuanshi;  // 定义解析出的32位无符号整数
extern volatile uint8_t dataReady; // 数据准备标志

#endif


