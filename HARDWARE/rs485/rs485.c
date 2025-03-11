#include "sys.h"		    
#include "rs485.h"	 
#include "usart.h"	 
#include "delay.h"
#include "led.h"
#include "stdarg.h"	 	 
#include "stdio.h"	
#include "string.h"
#include "4G.h"
//#include "flash.h"
//接收缓存区 	
u8 UART4_RX_BUF[UART4_REC_LEN];  	//接收缓冲,最大64个字节.
u8 UART4_TX_BUF[UART4_SEND_LEN];     //接收缓冲,最大USART_REC_LEN个字节.	  
  
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
volatile uint8_t dataReady = 0; // 数据准备标志
u16 UART4_RX_STA=0;       //接收状态标记
uint32_t value_yuanshi;  // 定义解析出的32位无符号整数
uint16_t value_shiji;
uint16_t last_value;  // 定义解析出的16位无符号整数
double value_xishu = 16.525;    //转换系数
uint8_t UART4_RX_Index = 0;  // 当前接收字节的索引
double value_yuliang = 0;    //调整余量

//初始化IO 串口2
//pclk1:PCLK1时钟频率(Mhz)
//bound:波特率	  
void RS485_Init(u32 bound)
{  
	    //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC| RCC_APB2Periph_AFIO, ENABLE);	//使能PORTC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE); //使能UART4
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//要先开时钟，再重映射；这句表示关闭jtag，使能swd。
 	USART_DeInit(UART4);  //复位串口4

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				 //PB8端口配置
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);

    //UART4_TX   PC.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PC.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOC, &GPIO_InitStructure); //初始化PC10
 
	//UART4_RX	  PC.11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOC, &GPIO_InitStructure);  //初始化PC11    

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4,ENABLE);//复位串口2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4,DISABLE);//停止复位

	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据长度
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;///奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发模式

  USART_Init(UART4, &USART_InitStructure); ; //初始化串口
  
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn; //使能串口2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //从优先级2级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能外部中断通道
	NVIC_Init(&NVIC_InitStructure); //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
 
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启中断
   
  USART_Cmd(UART4, ENABLE);                    //使能串口 
  RS485_TX_EN=0;			//默认为接收模式
 
}


#if EN_UART4_RX   //如果使能了接收
void UART4_IRQHandler(void) // 串口4中断服务程序
{
    uint8_t Res;

    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  // 检查接收中断
    {
        Res = USART_ReceiveData(UART4);  // 读取接收到的数据

        // 将接收到的字节存储到缓冲区
        UART4_RX_BUF[UART4_RX_Index++] = Res;
  	// 检查是否接收到完整的9字节数据包
        if (UART4_RX_Index >= 9) {
            // 解析接收到的数据包
			dataReady = 1;
            if (UART4_RX_BUF[0] == 0x01 && UART4_RX_BUF[1] == 0x03 && UART4_RX_BUF[2] == 0x04) {
                // 提取数据的第4到第7字节，并将其转换为32位无符号整数
                value_yuanshi = (UART4_RX_BUF[3] << 24) | (UART4_RX_BUF[4] << 16) | 
                        (UART4_RX_BUF[5] << 8) | UART4_RX_BUF[6];

				value_shiji = 4000 - value_yuanshi/value_xishu + value_yuliang - 320;
                // 输出数据，可以通过串口或其他方式打印
				printf("Parsed Data: %u\n", value_shiji);
            }

            // 重置索引，准备接收下一个9字节的数据包
            UART4_RX_Index = 0;
        }

        // 清除中断标志
        USART_ClearITPendingBit(UART4, USART_IT_RXNE);  
    }
}

/*************************校验码计算函数*************************/
unsigned int Crc_Count(unsigned char pbuf[],unsigned char num)
{
   int i,j;
   unsigned int wcrc=0xffff;
   for(i=0;i<num;i++)
   {
     wcrc^=(unsigned int)(pbuf[i]);
		 for (j=0;j<8;j++)
		 {
				if(wcrc&0x0001)
			{
				 wcrc>>=1;
				 wcrc^=0xa001;
			}
			else wcrc>>=1;		
		 }
   }   
   return wcrc;
}
	//串口4,printf 函数
//确保一次发送数据不超过USART3_MAX_SEND_LEN字节
void printDeviceDataInHex(unsigned char* buffer, int length) {
    int i;
    printf("Device_Data:");

	for (i = 0; i < length; i++) {
        printf("%02X ", buffer[i]);  // 将每个字节格式化为两位的16进制数
    }
    printf("\r\n");
}

void u4_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	RS485_TX_EN=1;			//默认为接收模式
 
	va_start(ap,fmt);
	vsprintf((char*)UART4_RX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)UART4_RX_BUF);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
	  while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
		USART_SendData(UART4,UART4_RX_BUF[j]); 
	} 
	RS485_TX_EN=0;			//默认为接收模式
 
}

/**********************************发送函数************************************************************/
void USART_Transmit(USART_TypeDef* USARTx,u8 *data,int num)	 //发送数组
{
	while(num--)
	{
		while(USART_GetFlagStatus(USARTx,USART_FLAG_TXE)==RESET);//发送寄存器空
		USART_SendData(USARTx,*data++);//发送单个数据//USART1->DR=*data++;
		while(USART_GetFlagStatus(USARTx,USART_FLAG_TC)==RESET);//等待数据帧发送完毕
	}
}
void RS485_SendStr(char*SendBuf)//串口4打印数据
{
	while(*SendBuf)
	{
		while((UART4->SR&0X40)==0);//等待发送完成 
		UART4->DR = (u8) *SendBuf; 
		SendBuf++;
	}
}

void USART_Transmit1(USART_TypeDef* USARTx,u8 data)//发送一个字节
{ 
   while(USART_GetFlagStatus(USARTx,USART_FLAG_TXE)==RESET);//发送寄存器空
	 USART_SendData(USARTx,data);//发送单个数据//USART1->DR=*data++;
	 while(USART_GetFlagStatus(USARTx,USART_FLAG_TC)==RESET);//等待数据帧发送完毕
}
#endif	


































//#include "rs485.h"
//#include "delay.h"
//#include "usart.h"	


///*******************************************************************************
//* 函 数 名         : RS485_Init
//* 函数功能		   : USART2初始化函数
//* 输    入         : bound:波特率
//* 输    出         : 无
//*******************************************************************************/  
//void RS485_Init(u32 bound)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC,ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能UART4时钟
//	
//	/*  配置GPIO的模式和IO口 */
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;	//TX-485	//串口输出PC10
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;		  //复用推挽输出
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	
//	GPIO_Init(GPIOC,&GPIO_InitStructure);		/* 初始化串口输入IO */
//	
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;	//RX-485	   //串口输入PC11
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;	    //模拟输入
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;	//CS-485
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	   //推挽输出
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB,&GPIO_InitStructure);
//	
//	//USART4 初始化设置
//	USART_InitStructure.USART_BaudRate = bound;//波特率设置
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
//	USART_Init(UART4, &USART_InitStructure); //初始化串口2
//	
//	USART_Cmd(UART4, ENABLE);  //使能串口 4
//	
//	USART_ClearFlag(UART4, USART_FLAG_TC);
//		
//	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启接受中断

//	//Uart4 NVIC 配置
//	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//子优先级2
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

//}

//void RS485_SendStr(char*SendBuf)//串口4打印数据
//{
//	while(*SendBuf)
//	{
//		while((UART4->SR&0X40)==0);//等待发送完成 
//		UART4->DR = (u8) *SendBuf; 
//		SendBuf++;
//	}
//}

//UART4_BUF Uart4_buff;

//void UART4_receive_process_event(char ch )     //串口2给4g用
//{
//    if(Uart4_buff.index >= BUFLEN)
//    {
//        Uart4_buff.index = 0 ;
//    }
//    else
//    {
//        Uart4_buff.buf[Uart4_buff.index++] = ch;
//    }
//}


//void Clear_usrt4buf(void)
//{
//	Uart4_buff.index = 0 ;
//	memset(Uart4_buff.buf,0,sizeof(Uart4_buff.buf));
//}

//void UART4_IRQHandler(void)
//{
//	char Res;
//	Res=Res;
//	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  //接收中断，可以扩展来控制
//	{
//			Res=USART_ReceiveData(UART4);//接收模块的数据;
//			UART4_receive_process_event(Res);//接收模块的数据
//	} 
//	if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)  //模块空闲
//	{
//			Res=USART_ReceiveData(UART4);//接收模块的数据;
//	} 
//}




