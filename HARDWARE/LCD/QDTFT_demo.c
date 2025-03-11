/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "Lcd_Driver.h"
#include "GUI.h"
#include "delay.h"
#include "QDTFT_demo.h"

unsigned char Num[10]={9,8,7,6,5,4,3,2,1,0};
void Redraw_Mainmenu(void)
{

	Lcd_Clear(GRAY0);
	
	Gui_DrawFont_GBK16(16,0,BLUE,GRAY0,"ȫ�����Ӽ���");
	Gui_DrawFont_GBK16(16,20,RED,GRAY0,"Һ�����Գ���");

	DisplayButtonUp(15,38,113,58); //x1,y1,x2,y2
	Gui_DrawFont_GBK16(16,40,YELLOW,GRAY0,"��ɫ������");

	DisplayButtonUp(15,68,113,88); //x1,y1,x2,y2
	Gui_DrawFont_GBK16(16,70,BLUE,GRAY0,"������ʾ����");

	DisplayButtonUp(15,98,113,118); //x1,y1,x2,y2
	Gui_DrawFont_GBK16(16,100,RED,GRAY0,"ͼƬ��ʾ����");
	delay_ms(1500);
}

void Num_Test(void)
{
	u8 i=0;
	Lcd_Clear(GRAY0);
	Gui_DrawFont_GBK16(0,20,RED,GRAY0,"4G MODULE START.");
	delay_ms(1000);
	Lcd_Clear(GRAY0);

	for(i=0;i<10;i++)
	{
	Gui_DrawFont_Num32((i%3)*40,32*(i/3)+5,RED,GRAY0,Num[i]);
	delay_ms(100);
	}
	
}

void Font_Test(void)
{
	//Lcd_Clear(GRAY0);
	//Gui_DrawFont_GBK16(16,10,BLUE,GRAY0,"������ʾ����");
	
	//delay_ms(1000);
	//Lcd_Clear(GRAY0);
	Gui_DrawFont_GBK16(16,30,BLUE,WHITE,"STM32 4G GPS");
	Gui_DrawFont_GBK16(16,50,BLUE,WHITE,"TCP/COAP/MQTT");
	Gui_DrawFont_GBK16(16,70,RED,WHITE, "�������̴�ѧ");
}

void Display_Menu(void)
{
	Gui_DrawFont_GBK16(10,70,GREEN,WHITE, "XPU-207-Lab");
}

void Color_Test(void)
{
	u8 i=1;
	Lcd_Clear(GRAY0);
	
	Gui_DrawFont_GBK16(20,10,BLUE,GRAY0,"Color Test");
	delay_ms(200);

	while(i--)
	{
			Lcd_Clear(WHITE);
			Lcd_Clear(BLACK);
			Lcd_Clear(RED);
	  	Lcd_Clear(GREEN);
	  	Lcd_Clear(BLUE);
	}		
}

//ȡģ��ʽ ˮƽɨ�� ������ ��λ��ǰ
void showimage(const unsigned char *p) //��ʾ40*40 QQͼƬ
{
  	int i,j,k; 
	unsigned char picH,picL;
	Lcd_Clear(WHITE); //����  
	
	for(k=2;k<4;k++)
	{
	   	for(j=0;j<3;j++)
		{	
			Lcd_SetRegion(40*j,40*k,40*j+39,40*k+39);		//��������
		    for(i=0;i<40*40;i++)
			 {	
			 	picL=*(p+i*2);	//���ݵ�λ��ǰ
				picH=*(p+i*2+1);				
				LCD_WriteData_16Bit(picH<<8|picL);  						
			 }	
		 }
	}		
}
/*
void QDTFT_Test_Demo(void)
{
	Lcd_Init();
	LCD_LED_SET;//ͨ��IO���Ʊ�����				
	//Redraw_Mainmenu();//�������˵�(�����������ڷֱ��ʳ�������ֵ�����޷���ʾ)
	//Color_Test();//�򵥴�ɫ������
	//Num_Test();//������������
		
	//showimage(gImage_qq);//ͼƬ��ʾʾ��
	Font_Test();//��Ӣ����ʾ����	
	delay_ms(1200);
	LCD_LED_CLR;//IO���Ʊ�����	
}
*/









