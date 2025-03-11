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
//���ջ����� 	
u8 UART4_RX_BUF[UART4_REC_LEN];  	//���ջ���,���64���ֽ�.
u8 UART4_TX_BUF[UART4_SEND_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.	  
  
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
volatile uint8_t dataReady = 0; // ����׼����־
u16 UART4_RX_STA=0;       //����״̬���
uint32_t value_yuanshi;  // �����������32λ�޷�������
uint16_t value_shiji;
uint16_t last_value;  // �����������16λ�޷�������
double value_xishu = 16.525;    //ת��ϵ��
uint8_t UART4_RX_Index = 0;  // ��ǰ�����ֽڵ�����
double value_yuliang = 0;    //��������

//��ʼ��IO ����2
//pclk1:PCLK1ʱ��Ƶ��(Mhz)
//bound:������	  
void RS485_Init(u32 bound)
{  
	    //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC| RCC_APB2Periph_AFIO, ENABLE);	//ʹ��PORTCʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE); //ʹ��UART4
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//Ҫ�ȿ�ʱ�ӣ�����ӳ�䣻����ʾ�ر�jtag��ʹ��swd��
 	USART_DeInit(UART4);  //��λ����4

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				 //PB8�˿�����
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);

    //UART4_TX   PC.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PC.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOC, &GPIO_InitStructure); //��ʼ��PC10
 
	//UART4_RX	  PC.11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOC, &GPIO_InitStructure);  //��ʼ��PC11    

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4,ENABLE);//��λ����2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4,DISABLE);//ֹͣ��λ

	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ���ݳ���
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;///��żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�ģʽ

  USART_Init(UART4, &USART_InitStructure); ; //��ʼ������
  
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn; //ʹ�ܴ���2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure); //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
 
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//�����ж�
   
  USART_Cmd(UART4, ENABLE);                    //ʹ�ܴ��� 
  RS485_TX_EN=0;			//Ĭ��Ϊ����ģʽ
 
}


#if EN_UART4_RX   //���ʹ���˽���
void UART4_IRQHandler(void) // ����4�жϷ������
{
    uint8_t Res;

    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  // �������ж�
    {
        Res = USART_ReceiveData(UART4);  // ��ȡ���յ�������

        // �����յ����ֽڴ洢��������
        UART4_RX_BUF[UART4_RX_Index++] = Res;
  	// ����Ƿ���յ�������9�ֽ����ݰ�
        if (UART4_RX_Index >= 9) {
            // �������յ������ݰ�
			dataReady = 1;
            if (UART4_RX_BUF[0] == 0x01 && UART4_RX_BUF[1] == 0x03 && UART4_RX_BUF[2] == 0x04) {
                // ��ȡ���ݵĵ�4����7�ֽڣ�������ת��Ϊ32λ�޷�������
                value_yuanshi = (UART4_RX_BUF[3] << 24) | (UART4_RX_BUF[4] << 16) | 
                        (UART4_RX_BUF[5] << 8) | UART4_RX_BUF[6];

				value_shiji = 4000 - value_yuanshi/value_xishu + value_yuliang - 320;
                // ������ݣ�����ͨ�����ڻ�������ʽ��ӡ
				printf("Parsed Data: %u\n", value_shiji);
            }

            // ����������׼��������һ��9�ֽڵ����ݰ�
            UART4_RX_Index = 0;
        }

        // ����жϱ�־
        USART_ClearITPendingBit(UART4, USART_IT_RXNE);  
    }
}

/*************************У������㺯��*************************/
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
	//����4,printf ����
//ȷ��һ�η������ݲ�����USART3_MAX_SEND_LEN�ֽ�
void printDeviceDataInHex(unsigned char* buffer, int length) {
    int i;
    printf("Device_Data:");

	for (i = 0; i < length; i++) {
        printf("%02X ", buffer[i]);  // ��ÿ���ֽڸ�ʽ��Ϊ��λ��16������
    }
    printf("\r\n");
}

void u4_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	RS485_TX_EN=1;			//Ĭ��Ϊ����ģʽ
 
	va_start(ap,fmt);
	vsprintf((char*)UART4_RX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)UART4_RX_BUF);		//�˴η������ݵĳ���
	for(j=0;j<i;j++)							//ѭ����������
	{
	  while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
		USART_SendData(UART4,UART4_RX_BUF[j]); 
	} 
	RS485_TX_EN=0;			//Ĭ��Ϊ����ģʽ
 
}

/**********************************���ͺ���************************************************************/
void USART_Transmit(USART_TypeDef* USARTx,u8 *data,int num)	 //��������
{
	while(num--)
	{
		while(USART_GetFlagStatus(USARTx,USART_FLAG_TXE)==RESET);//���ͼĴ�����
		USART_SendData(USARTx,*data++);//���͵�������//USART1->DR=*data++;
		while(USART_GetFlagStatus(USARTx,USART_FLAG_TC)==RESET);//�ȴ�����֡�������
	}
}
void RS485_SendStr(char*SendBuf)//����4��ӡ����
{
	while(*SendBuf)
	{
		while((UART4->SR&0X40)==0);//�ȴ�������� 
		UART4->DR = (u8) *SendBuf; 
		SendBuf++;
	}
}

void USART_Transmit1(USART_TypeDef* USARTx,u8 data)//����һ���ֽ�
{ 
   while(USART_GetFlagStatus(USARTx,USART_FLAG_TXE)==RESET);//���ͼĴ�����
	 USART_SendData(USARTx,data);//���͵�������//USART1->DR=*data++;
	 while(USART_GetFlagStatus(USARTx,USART_FLAG_TC)==RESET);//�ȴ�����֡�������
}
#endif	


































//#include "rs485.h"
//#include "delay.h"
//#include "usart.h"	


///*******************************************************************************
//* �� �� ��         : RS485_Init
//* ��������		   : USART2��ʼ������
//* ��    ��         : bound:������
//* ��    ��         : ��
//*******************************************************************************/  
//void RS485_Init(u32 bound)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC,ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//ʹ��UART4ʱ��
//	
//	/*  ����GPIO��ģʽ��IO�� */
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;	//TX-485	//�������PC10
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;		  //�����������
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	
//	GPIO_Init(GPIOC,&GPIO_InitStructure);		/* ��ʼ����������IO */
//	
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;	//RX-485	   //��������PC11
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;	    //ģ������
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;	//CS-485
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	   //�������
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB,&GPIO_InitStructure);
//	
//	//USART4 ��ʼ������
//	USART_InitStructure.USART_BaudRate = bound;//����������
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
//	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
//	USART_Init(UART4, &USART_InitStructure); //��ʼ������2
//	
//	USART_Cmd(UART4, ENABLE);  //ʹ�ܴ��� 4
//	
//	USART_ClearFlag(UART4, USART_FLAG_TC);
//		
//	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//���������ж�

//	//Uart4 NVIC ����
//	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//�����ȼ�2
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

//}

//void RS485_SendStr(char*SendBuf)//����4��ӡ����
//{
//	while(*SendBuf)
//	{
//		while((UART4->SR&0X40)==0);//�ȴ�������� 
//		UART4->DR = (u8) *SendBuf; 
//		SendBuf++;
//	}
//}

//UART4_BUF Uart4_buff;

//void UART4_receive_process_event(char ch )     //����2��4g��
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
//	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  //�����жϣ�������չ������
//	{
//			Res=USART_ReceiveData(UART4);//����ģ�������;
//			UART4_receive_process_event(Res);//����ģ�������
//	} 
//	if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)  //ģ�����
//	{
//			Res=USART_ReceiveData(UART4);//����ģ�������;
//	} 
//}




