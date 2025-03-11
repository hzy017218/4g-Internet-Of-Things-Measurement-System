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
u16 UART4_RX_STA=0;       //����״̬���
uint8_t UART4_RX_Index = 0;  // ��ǰ�����ֽڵ�����


uint8_t data1[8]={0X01,0X03,0X00,0X32,0X00,0X02,0X65,0XC4};
uint8_t data2[8]={0X01,0X03,0X00,0X3A,0X00,0X01,0XA4,0X07};
uint8_t data3[8]={0X01,0X03,0X00,0X3D,0X00,0X01,0X15,0XC6};
uint8_t data4[8]={0X01,0X04,0X00,0X00,0X00,0X03,0XB0,0X0B};

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


static void Delay(__IO uint32_t nCount)	 //�򵥵���ʱ����
{
	for(; nCount != 0; nCount--);
}

/***************** ����һ���ַ�  **********************/
//ʹ�õ��ֽ����ݷ���ǰҪʹ�ܷ������ţ����ͺ�Ҫʹ�ܽ������š�
void _485_SendByte(  uint8_t ch )
{
	/* ����һ���ֽ����ݵ�USART1 */
	USART_SendData(UART4,ch);
		
	/* �ȴ�������� */
	while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);	
	
}
/*****************  ����ָ�����ȵ��ַ��� **********************/
void _485_SendStr_length( uint8_t *str,uint32_t strlen )
{
	unsigned int k=0;

	_485_TX_EN()	;//	ʹ�ܷ�������	
    do 
    {
        _485_SendByte( *(str + k) );
        k++;
    } while(k < strlen);
		
	/*�Ӷ�����ʱ����֤485�����������*/
	Delay(0x10FF);
		
	_485_RX_EN()	;//	ʹ�ܽ�������
}


/*****************  �����ַ��� **********************/
void _485_SendString(  uint8_t *str)
{
	unsigned int k=0;
	
	_485_TX_EN()	;//	ʹ�ܷ�������
	
    do 
    {
        _485_SendByte(  *(str + k) );
        k++;
    } while(*(str + k)!='\0');
	
	/*�Ӷ�����ʱ����֤485�����������*/
	Delay(0xFFF);
		
	_485_RX_EN()	;//	ʹ�ܽ�������
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






//�жϻ��洮������
#define UART_BUFF_SIZE      1024
volatile    uint16_t uart_p = 0;
uint8_t     uart_buff[UART_BUFF_SIZE];
char *pbuf; 
int data_t;
int last_temp = 0;

void UART4_IRQHandler(void) // ����4�жϷ������
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

//��ȡ���յ������ݺͳ���
char *get_rebuff(uint16_t *len) 
{
    *len = uart_p;
    return (char *)&uart_buff;
}

//��ջ�����
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
//		printf("��ȡ��16�����ַ���: %04x\n", hex2 );
//		printf("ת��Ϊʮ����: %f\n", dec2);
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
//    uint16_t hex;  // ���ڴ洢��ȡ����λ16����������Ҫ3���ֽ����洢�ַ���
//    float dec;
//    float data;

//    pbuf = get_rebuff(&len);

//    if(len >= 11)  // ȷ�����յ������ݳ����㹻
//    {
//      if(pbuf[0]==0x01&&pbuf[1]==0x04&&pbuf[2]==0x06)
//      {
//        hex = (pbuf[3]<<8) + pbuf[4];    
//        //_485_DEBUG_ARRAY((uint8_t*)pbuf,len);        
//        dec = (float)hex;
////        printf("��ȡ��16�����ַ���: %04x\n", hex);
////        printf("ת��Ϊʮ����: %f\n", dec);                
//        data = dec/249;
////        printf("�������Ϊ: %f mA \n", data);                
//        data_t = (int)((((data-4)/16)*140)-40)+1.3;                
////        printf("����¶�Ϊ: %d C \n", data_t);
//        clean_rebuff();
//      }

//    }
//    return data_t;
//}

























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




