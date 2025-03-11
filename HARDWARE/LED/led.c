#include "led.h"
#include "delay.h"

//��ʼ��PA15 PA1   
//LED IO��ʼ��
void LED_Init(void)
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
	//PB3 PB4 ��Ҫ���ø���
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);	//��GPIOʹ��ʱ�ӣ��ȿ����ù���
 GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//ӳ��PB3 PB4 �ڵ�GPIO���� 
	

		//LED1-->PB5 LED2-->PB4 LED3-->PB3
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_4|GPIO_Pin_3;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�����
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //Ƶ��
 GPIO_Init(GPIOB, &GPIO_InitStructure);	  				 //��ʼ��PB��
	//PB5 PB4 PB3
 GPIO_SetBits(GPIOB,GPIO_Pin_5|GPIO_Pin_4|GPIO_Pin_3); 	//�رյ� 			

}
 

/*****************************************************
�����
*****************************************************/
void LED_Run(void)
{
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_5); 	
	delay_ms(300);
	GPIO_ResetBits(GPIOB,GPIO_Pin_4); 	
	delay_ms(300);
	GPIO_ResetBits(GPIOB,GPIO_Pin_3); 	
	delay_ms(300);
	GPIO_SetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);
}
 

/*****************************************************
����ģ�� �͵�ƽ��ģ����й��� �ߵ�ƽ����ģ����й���
*****************************************************/
void CSTX_4GCTR_Init(void)
{  
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��PC�˿�ʱ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 		//PA12ֱ�ӿ���ģ����й���˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);		    //PC3 �����
	
		GPIO_SetBits(GPIOA,GPIO_Pin_12); 	
		delay_ms(1000);
		GPIO_ResetBits(GPIOA,GPIO_Pin_12); 	 
}









