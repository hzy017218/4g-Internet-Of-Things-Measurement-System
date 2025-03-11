#ifndef __USART4_H
#define __USART4_H			 
#include "sys.h"	 								  

#define RS485_TX_EN		PBout(8)	//485模式控制.0,接收;1,发送.
#define UART4_REC_LEN  			9  
#define UART4_SEND_LEN		  600					
#define EN_UART4_RX 			  1		

extern int data_t;
extern int last_temp;
extern uint8_t data1[8];
extern uint8_t data2[8];
extern uint8_t data3[8];
extern uint8_t data4[8];

	/// 不精确的延时
static void _485_delay(__IO u32 nCount)
{
	for(; nCount != 0; nCount--);
} 


#define _485_DEBUG_ON          1
#define _485_DEBUG_ARRAY_ON   1
#define _485_DEBUG_FUNC_ON    1

/*控制收发引脚*/
//进入接收模式,必须要有延时等待485处理完数据
#define _485_RX_EN()			_485_delay(1000); GPIO_ResetBits(GPIOB,GPIO_Pin_8);  _485_delay(1000);
//进入发送模式,必须要有延时等待485处理完数据
#define _485_TX_EN()			_485_delay(1000); GPIO_SetBits(GPIOB,GPIO_Pin_8);  _485_delay(1000);


#define _485_DEBUG_ARRAY(array, num)    do{\
                                         int32_t i;\
                                         uint8_t* a = array;\
                                         if(_485_DEBUG_ARRAY_ON)\
                                         {\
                                            printf("<<-_485-DEBUG-ARRAY->>\n");\
                                            for (i = 0; i < (num); i++)\
                                            {\
                                                printf("%02x   ", (a)[i]);\
                                                if ((i + 1 ) %10 == 0)\
                                                {\
                                                    printf("\n");\
                                                }\
                                            }\
                                            printf("\n");\
                                        }\
                                       }while(0)

#define _485_DEBUG_FUNC()               do{\
                                         if(_485_DEBUG_FUNC_ON)\
                                         printf("<<-_485-FUNC->> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)

	  	
extern u8  UART4_RX_BUF[UART4_REC_LEN]; 
extern u8  UART4_TX_BUF[UART4_SEND_LEN]; 		
extern u16 UART4_RX_STA;  
extern uint8_t UART4_RX_Index;  

void RS485_Init(u32 bound);
//void RS485_SendStr(char*SendBuf);
//void u4_printf(char* fmt,...);
//void USART_Transmit(USART_TypeDef* USARTx,u8 *data,int num);
//extern void printDeviceDataInHex(unsigned char* buffer, int length);
//unsigned int Crc_Count(unsigned char pbuf[],unsigned char num);
//void ProcessReceivedData(void);

void _485_SendByte(  uint8_t ch );
void _485_SendStr_length( uint8_t *str,uint32_t strlen );
void _485_SendString(  uint8_t *str);
void RS485_SendStr(char*SendBuf);
void USART_Transmit(USART_TypeDef* USARTx,u8 *data,int num);

char *get_rebuff(uint16_t *len);
void clean_rebuff(void);

void PT100_init(void);
void Get_PT100_data(void);
//int PT100_data (void);


 //定义结构体
typedef struct {
    int temperature1;  // 接收3，4
    int temperature2;  // 接收5，6
} TemperatureData;

extern TemperatureData PT100_data(void);

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
