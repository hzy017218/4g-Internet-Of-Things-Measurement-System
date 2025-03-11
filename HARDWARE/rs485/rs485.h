#ifndef __USART4_H
#define __USART4_H			 
#include "sys.h"	 								  

#define RS485_TX_EN		PBout(8)	//485模式控制.0,接收;1,发送.
#define UART4_REC_LEN  			9  
#define UART4_SEND_LEN		  600					
#define EN_UART4_RX 			  1		
	  	
extern u8  UART4_RX_BUF[UART4_REC_LEN]; 
extern u8  UART4_TX_BUF[UART4_SEND_LEN]; 		
extern u16 UART4_RX_STA;  
extern uint8_t UART4_RX_Index;  

extern double value_xishu;
extern double value_yuliang;
extern uint16_t value_shiji;
extern uint16_t last_value;
extern uint32_t value_yuanshi;  // 定义解析出的32位无符号整数
extern volatile uint8_t dataReady; // 数据准备标志

void RS485_Init(u32 bound);
void RS485_SendStr(char*SendBuf);
void u4_printf(char* fmt,...);
void USART_Transmit(USART_TypeDef* USARTx,u8 *data,int num);
extern void printDeviceDataInHex(unsigned char* buffer, int length);
unsigned int Crc_Count(unsigned char pbuf[],unsigned char num);
void ProcessReceivedData(void);

#endif	   


























//#ifndef _rs485_H
//#define _rs485_H

//#include "sys.h"
//#include "stdio.h"
//#include "string.h"

//#define BUF4LEN 256      //数组缓存大小
//typedef struct _UART4_BUF
//{
//    char buf [BUF4LEN+1];               
//    unsigned int index ;
//}UART4_BUF;	


////模式控制
//#define RS485_TX_EN		PBout(8)	//485模式控制.0,接收;1,发送.
//														 
//void RS485_Init(u32 bound);
//void Clear_usrt4buf(void);
//void RS485_SendStr(char*SendBuf);

//extern UART4_BUF Uart4_buff;

//#endif
