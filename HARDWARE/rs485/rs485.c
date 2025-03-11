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
u16 UART4_RX_STA=0;       //接收状态标记
uint8_t UART4_RX_Index = 0;  // 当前接收字节的索引


uint8_t data1[8]={0X01,0X03,0X00,0X32,0X00,0X02,0X65,0XC4};
uint8_t data2[8]={0X01,0X03,0X00,0X3A,0X00,0X01,0XA4,0X07};
uint8_t data3[8]={0X01,0X03,0X00,0X3D,0X00,0X01,0X15,0XC6};
uint8_t data4[8]={0X01,0X04,0X00,0X00,0X00,0X03,0XB0,0X0B};

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


static void Delay(__IO uint32_t nCount)	 //简单的延时函数
{
	for(; nCount != 0; nCount--);
}

/***************** 发送一个字符  **********************/
//使用单字节数据发送前要使能发送引脚，发送后要使能接收引脚。
void _485_SendByte(  uint8_t ch )
{
	/* 发送一个字节数据到USART1 */
	USART_SendData(UART4,ch);
		
	/* 等待发送完毕 */
	while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);	
	
}
/*****************  发送指定长度的字符串 **********************/
void _485_SendStr_length( uint8_t *str,uint32_t strlen )
{
	unsigned int k=0;

	_485_TX_EN()	;//	使能发送数据	
    do 
    {
        _485_SendByte( *(str + k) );
        k++;
    } while(k < strlen);
		
	/*加短暂延时，保证485发送数据完毕*/
	Delay(0x10FF);
		
	_485_RX_EN()	;//	使能接收数据
}


/*****************  发送字符串 **********************/
void _485_SendString(  uint8_t *str)
{
	unsigned int k=0;
	
	_485_TX_EN()	;//	使能发送数据
	
    do 
    {
        _485_SendByte(  *(str + k) );
        k++;
    } while(*(str + k)!='\0');
	
	/*加短暂延时，保证485发送数据完毕*/
	Delay(0xFFF);
		
	_485_RX_EN()	;//	使能接收数据
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






//中断缓存串口数据
#define UART_BUFF_SIZE      1024
volatile    uint16_t uart_p = 0;
uint8_t     uart_buff[UART_BUFF_SIZE];
char *pbuf; 
int data_t;
int last_temp = 0;

void UART4_IRQHandler(void) // 串口4中断服务程序
{

    if(uart_p<UART_BUFF_SIZE)
    {
        if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
        {
            uart_buff[uart_p] = USART_ReceiveData(UART4);
            uart_p++;

						USART_ClearITPendingBit(UART4, USART_IT_RXNE);
        }
    }
		else
		{
			USART_ClearITPendingBit(UART4, USART_IT_RXNE);
			clean_rebuff();       
		}
}

//获取接收到的数据和长度
char *get_rebuff(uint16_t *len) 
{
    *len = uart_p;
    return (char *)&uart_buff;
}

//清空缓冲区
void clean_rebuff(void) 
{

    uint16_t i=UART_BUFF_SIZE+1;
    uart_p = 0;
	while(i)
		uart_buff[--i]=0;

}


void PT100_init(void)
{
    _485_SendStr_length(data1, 8);
    _485_SendStr_length(data2, 8);
    _485_SendStr_length(data3, 8);
	
}

void Get_PT100_data(void)
{
    _485_SendStr_length(data4, 8);
}

TemperatureData PT100_data(void)
{
    uint16_t len;
    uint16_t hex1, hex2;  
    float dec1, dec2;
    float data1, data2;
    TemperatureData result_temp = {0, 0};  

    pbuf = get_rebuff(&len);

    if(len >= 11)  
    {
        if(pbuf[0]==0x01&&pbuf[1]==0x04&&pbuf[2]==0x06)
        {
            hex1 = (pbuf[3]<<8) + pbuf[4];    
            dec1 = (float)hex1;
            data1 = dec1/249;
            result_temp.temperature1 = (int)((((data1-4)/16)*140)-40)+1.3;

            hex2 = (pbuf[5]<<8) + pbuf[6];    
            dec2 = (float)hex2;
//		printf("提取的16进制字符串: %04x\n", hex2 );
//		printf("转换为十进制: %f\n", dec2);
            data2 = dec2/249;
            result_temp.temperature2 = (int)((((data2-4)/16)*140)-40)+1.3;

            clean_rebuff();
        }
    }
    return result_temp;  
}



//int PT100_data (void)
//{
//    uint16_t len;
//    uint16_t hex;  // 用于存储提取的两位16进制数，需要3个字节来存储字符串
//    float dec;
//    float data;

//    pbuf = get_rebuff(&len);

//    if(len >= 11)  // 确保接收到的数据长度足够
//    {
//      if(pbuf[0]==0x01&&pbuf[1]==0x04&&pbuf[2]==0x06)
//      {
//        hex = (pbuf[3]<<8) + pbuf[4];    
//        //_485_DEBUG_ARRAY((uint8_t*)pbuf,len);        
//        dec = (float)hex;
////        printf("提取的16进制字符串: %04x\n", hex);
////        printf("转换为十进制: %f\n", dec);                
//        data = dec/249;
////        printf("输出电流为: %f mA \n", data);                
//        data_t = (int)((((data-4)/16)*140)-40)+1.3;                
////        printf("输出温度为: %d C \n", data_t);
//        clean_rebuff();
//      }

//    }
//    return data_t;
//}

























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




