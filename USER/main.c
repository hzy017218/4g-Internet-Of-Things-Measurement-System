#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"	 
#include "math.h"			
#include "stdio.h"
#include "stm32f10x_flash.h"
#include "stdlib.h"
#include "string.h"
#include "wdg.h"
#include "timer.h"
#include "stm32f10x_tim.h"
#include "4G.h"	
#include "rs485.h"
#include "adc.h"
///////////////������Һ����ͷ�ļ�/////////////////////
#include "Lcd_Driver.h"
#include "GUI.h"
#include "delay.h"
#include "Picture.h"
#include "QDTFT_demo.h"
#include "dht11.h"
/////////////////////////////////////////////////////
//������ʼ
//������·������ ��ʪ�� ��GPS������λ��Ϣ
////////////////////////////////////////////////////
int main(void)
{
	char strValue[12]; // ���ڴ洢�ַ������㹻�������ֵ�ͽ�����
	char temp_disp[BUFLEN];	//�¶���ʾ
	char dht[8];
	int last_temp1 = 0;
    int last_temp2 = 0;
	TemperatureData temps;
	
//	u16 adcx1,adcx2; //ADC1��ADC2��ֵ
//	char adcValue[8];
	
	
	/**********************************��ʼ��************************************************************/
	
  delay_init();	    	 //��ʱ������ʼ��	  
  NVIC_Configuration(); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	CSTX_4GCTR_Init();        //��ʼ��CSTX_4G�Ĺ������� ��ģ����й���
  uart_init(115200);//����1��ʼ����������PC���д�ӡģ�鷵������
	Uart1_SendStr("UART1 Init Successful\r\n");
	uart2_init(115200);//��ʼ����EC200���Ӵ���	
	Uart2_SendStr("UART2 Init Successful\r\n");
	uart3_init(115200);
	Uart3_SendStr("UART3 Init Successful\r\n");
	RS485_Init(9600);
	RS485_SendStr("RS485 Init Successful\r\n");
	LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
	
	//////////������Һ������ʾ����///////////////////////////
	Lcd_Init();
	LCD_LED_SET;//ͨ��IO���Ʊ�����				
	Num_Test();//������������	
	showimage(gImage_qq);//ͼƬ��ʾʾ��
	Font_Test();//��Ӣ����ʾ����	
	delay_ms(1200);
	/////////////////////////////////////////////////////////////   
	LED_Run();					//��ʼ������� 
	Lcd_Clear(WHITE);
	Close_MQTT_aliyunStudio();//�ر���һ������
	CSTX_4G_Init();//���豸��ʼ��
	Connect_MQTT_aliyunStudio();//����һ������
	Start_GPS();//��㿪��GPS������λ��
	Clear_Buffer_UART1();
	Display_Menu();
	DHT11_Init();	//��ʼ����ʪ�� ��PA11
	Adc_Init();	  	//ADC��ʼ��
	TIM3_Int_Init(4999,7199);//100ms����һ��  
	PT100_init();
	
	
//	/**********************************��������************************************************************/
//	Get_PT100_data();
//	delay_ms(1200);
//	PT100_data();
//	memset(dht,0,8);
//	sprintf(dht,"%d",data_t);
//	SendData_aliyunStudio("temp",(char *)dht);//�����¶�����
//	
//	memset(temp_disp,0,BUFLEN);
//	sprintf(temp_disp,"temp1:%d,temp2:%d",data_t,DHT11_Data.humi_int);	//��ʪ�ȴ�ӡ������
//	Gui_DrawFont_GBK16(0,50,RED,WHITE, (u8*)temp_disp); //��ʪ����ʾ��Һ����	
	
	while(1)
	{  
	Control_Led();     //Զ������

	Get_PT100_data();
	delay_ms(500);  
//			PT100_data(); 
	temps = PT100_data();	  

	/**********************************ʵʱ����************************************************************/

	/***********************�߶�ʵʱ����******************************/

	if ((value_shiji - last_value >= 1) || (value_shiji - last_value <= -1)) 
	{
		// ���value_shiji �Ƿ����
		if (value_shiji > 65535) {
				value_shiji = 65535;  
		} 
		sprintf(strValue, "%u", value_shiji);
		SendData_aliyunStudio("485", (char *)strValue);
		
		printf("Parsed Data_Height: %u\n", value_shiji);
		last_value = value_shiji; 
		
		memset(strValue,0,BUFLEN);
		sprintf(strValue,"Height:%d",value_shiji);	//�߶ȴ�ӡ������
		Gui_DrawFont_GBK16(10,90,RED,WHITE,(u8*)strValue);
	}
			
		/***********************�¶�ʵʱ����***************************/
	
	if ((temps.temperature1 - last_temp1 >= 1) || (temps.temperature1 - last_temp1 <= -1)) {
		memset(dht, 0, 8);
		sprintf(dht, "%d", temps.temperature1);
		SendData_aliyunStudio("temp", (char *)dht);

		memset(temp_disp, 0, BUFLEN);
		sprintf(temp_disp, "Temp1:%d", temps.temperature1);
		Gui_DrawFont_GBK16(10, 110, RED, WHITE, (u8*)temp_disp);

		last_temp1 = temps.temperature1;
		printf("Parsed Data----------Temp1: %d\n", temps.temperature1);
	}

	if ((temps.temperature2 - last_temp2 >= 1) || (temps.temperature2 - last_temp2 <= -1)) {
		memset(dht, 0, 8);
		sprintf(dht, "%d", temps.temperature2);
		SendData_aliyunStudio("humi", (char *)dht);

		memset(temp_disp, 0, BUFLEN);
		sprintf(temp_disp, "Temp2:%d", temps.temperature2);
		Gui_DrawFont_GBK16(10, 130, RED, WHITE, (u8*)temp_disp);

		last_temp2 = temps.temperature2;
		printf("Parsed Data----------Temp2: %d\n", temps.temperature2);
	}
			
			
			
//			if ((data_t - last_temp >= 2) || (data_t - last_temp <= -2)) 
//			{

//				memset(dht, 0, 8);
//				sprintf(dht, "%d", data_t);
//				SendData_aliyunStudio("temp",(char *)dht);//�����¶�����
//				printf("Parsed Data��������Temp1: %d\n", data_t);  
//				last_temp = data_t;

//				memset(temp_disp, 0, BUFLEN);
//				sprintf(temp_disp, "Temp1:%d", data_t);
//				Gui_DrawFont_GBK16(0, 110, RED, WHITE, (u8*)temp_disp);
//			}

				
				
	/*********************************��ʱ����************************************************************/
	
	if(time3_5s >= 7200)	//��ʱ�� 60    ���ӷ���һ������
	{							
			/**********************�߶ȶ�ʱ����****************************/
		
			sprintf(strValue, "%u", value_shiji);
			SendData_aliyunStudio("485", (char *)strValue);
			printf("Parsed Data: %u\n", value_shiji);
			last_value = value_shiji; 
			
			memset(strValue,0,BUFLEN);
			sprintf(strValue,"Height:%d",value_shiji);	//�߶ȴ�ӡ������
			Gui_DrawFont_GBK16(10,90,RED,WHITE,(u8*)strValue);
			////////////////////////////////////////////////////////////////

			/**********************�¶ȶ�ʱ����******************************/
//							DHT11_Read_TempAndHumidity();
			Get_PT100_data();
			delay_ms(200);
			temps = PT100_data();

			memset(dht,0,8);
			sprintf(dht,"%d",temps.temperature1);
			SendData_aliyunStudio("temp",(char *)dht);//�����¶�1

			memset(dht,0,8);
			sprintf(dht,"%d",temps.temperature2);
			SendData_aliyunStudio("humi",(char *)dht);//�����¶�2
								
			memset(temp_disp,0,BUFLEN);
//			sprintf(temp_disp,"Temp1:%d Temp2:%d",temps.temperature1,temps.temperature2);	//��ʪ�ȴ�ӡ������
			sprintf(temp_disp,"Temp1:%d",temps.temperature1);	//��ʪ�ȴ�ӡ������
			Gui_DrawFont_GBK16(10,110,RED,WHITE, (u8*)temp_disp); //��ʪ����ʾ��Һ����	

			memset(temp_disp,0,BUFLEN);
			sprintf(temp_disp,"Temp2:%d",temps.temperature2);
			Gui_DrawFont_GBK16(10,130,RED,WHITE, (u8*)temp_disp); //��ʪ����ʾ��Һ����	
			////////////////////////////////////////////////////////////////
									

			LED1=!LED1; //ϵͳ����ָʾ��
		}
  }	 
}

						


//							//����adc1 adc2 //////////////////////////////////////////////////////////////////////
//							adcx1=Get_Adc_Average(ADC_Channel_10,10); //��ȡ�õ�PC0��ADC1��ֵ
//							adcx2=Get_Adc_Average(ADC_Channel_11,10);
//							printf("ADC1ԭʼ��ֵ��%d, ADC2ԭʼ��ֵ��%d\r\n",(int)((float)adcx1/4096*100),(int)((float)adcx2/4096*100)); //��ӡԭʼ�ɼ�������
//							memset(dht,0,8);	//��մ�����1����
//							sprintf(adcValue,"%d",(int)((float)adcx1/4096*100));
//							SendData_aliyunStudio("ADC1",adcValue);
//							memset(dht,0,8);	//��մ�����2����
//							sprintf(adcValue,"%d",(int)((float)adcx2/4096*100));
//							SendData_aliyunStudio("ADC2",adcValue);	
//							memset(temp_disp,0,BUFLEN);
//							sprintf(temp_disp,"ADC1:%2d,ADC2:%2d",(int)((float)adcx1/4096*100),(int)((float)adcx2/4096*100));	//��ʪ�ȴ�ӡ������
//							Gui_DrawFont_GBK16(0,130,BLACK,WHITE, (u8*)temp_disp); //��ʪ����ʾ��Һ����			
							//////////////////////////////////////////////////////////////////////////////////////////



