#include "key.h"
#include "delay.h"

/*******************************************************************************
* �� �� ��         : KEY_Init
* ��������		   : ������ʼ��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //����ṹ�����	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;	//��������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;	//��������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}

/*******************************************************************************
* �� �� ��         : KEY_Scan
* ��������		   : ����ɨ����
* ��    ��         : mode=0:���ΰ��°���
					 mode=1���������°���
* ��    ��         : 0��δ�а�������
					 KEY_UP��K_UP������
					 KEY_DOWN��K_DOWN������
					 KEY_LEFT��K_LEFT������
					 KEY_RIGHT��K_RIGHT������
*******************************************************************************/
u8 KEY_Scan(u8 mode)
{
	static u8 key=1;
	if(mode==1) //������������
	{
		key=1;
	}
	if(key==1&&(K1==1||K2==0)) //����һ����������
	{
		key=0;
		delay_ms(8);
		if(K1==1)
		{
			return Key1; 
		}
		else if(K2==0)
		{
			return Key2; 
		}
	}
	else if(K1==0 && K2==1)    //�ް�������
	{
		key=1;
	}
	return 0;
}

