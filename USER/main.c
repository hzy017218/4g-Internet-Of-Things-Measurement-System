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
//	char *gpsStr; 	//GPSָ���λ��
//	char gpsDatalat[64];
//	char gpsDatalon[64];
//	u16 adcx1,adcx2; //ADC1��ADC2��ֵ
//	char adcValue[8];

  delay_init();	    	 //��ʱ������ʼ��	  
  NVIC_Configuration(); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	CSTX_4GCTR_Init();        //��ʼ��CSTX_4G�Ĺ������� ��ģ����й���
  uart_init(115200);//����1��ʼ����������PC���д�ӡģ�鷵������
	Uart1_SendStr("UART1 Init Successful\r\n");
	uart2_init(115200);//��ʼ����EC200���Ӵ���	
	Uart2_SendStr("UART2 Init Successful\r\n");
	uart3_init(115200);
	Uart3_SendStr("UART3 Init Successful\r\n");
	RS485_Init(115200);
	RS485_SendStr("RS485 Init Successful\r\n");
	
	printf("\r\n ############ http://www.csgsm.com/ ############\r\n ############("__DATE__ " - " __TIME__ ")############\r\n"); 
	
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
	
	while(1)
  {  
//			gpsStr = NULL;
			Control_Led();
			if ((value_shiji - last_value >= 1) || (value_shiji - last_value <= -1)) 
				{
					sprintf(strValue, "%u", value_shiji);
					SendData_aliyunStudio("485", (char *)strValue);
					printf("Parsed Data: %u\n", value_shiji);
					last_value = value_shiji; 
					
					memset(strValue,0,BUFLEN);
					sprintf(strValue,"Tension:%d",value_shiji);	//�߶ȴ�ӡ������
					Gui_DrawFont_GBK16(0,90,RED,WHITE,(u8*)strValue);
				}
			if(time3_5s >= 7200)	//��ʱ�� 60    ���ӷ���һ������
			{							
					//�����Ǹ߶ȵķ���/////////////////////////////////////////////////////////////
							sprintf(strValue, "%u", value_shiji);
							SendData_aliyunStudio("485", (char *)strValue);
							printf("Parsed Data: %u\n", value_shiji);
							last_value = value_shiji; 
							
							memset(strValue,0,BUFLEN);
							sprintf(strValue,"Tension:%d",value_shiji);	//�߶ȴ�ӡ������
							Gui_DrawFont_GBK16(0,90,RED,WHITE,(u8*)strValue);
							////////////////////////////////////////////////////////////////
				
							//��������ʪ�ȵķ���//////////////////////////////////////////////////////////////
							DHT11_Read_TempAndHumidity();
							
							memset(dht,0,8);
							sprintf(dht,"%d",DHT11_Data.temp_int);
							SendData_aliyunStudio("temp",(char *)dht);//�����¶�����
							memset(dht,0,8);
							sprintf(dht,"%d",DHT11_Data.humi_int);
							SendData_aliyunStudio("humi",(char *)dht);//����ʪ������
												
							memset(temp_disp,0,BUFLEN);
							sprintf(temp_disp,"temp:%d,humi:%d",DHT11_Data.temp_int,DHT11_Data.humi_int);	//��ʪ�ȴ�ӡ������
							Gui_DrawFont_GBK16(0,50,RED,WHITE, (u8*)temp_disp); //��ʪ����ʾ��Һ����			
							////////////////////////////////////////////////////////////////
							
							
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

		}
  }	 
}

							//							memset(gpsDatalat,0,64);    //ά�����
//							memset(gpsDatalon,0,64);    //�������
//							gpsStr=Get_GPS_RMC(1);      //��ȡά�� ����Ǿ�����ƫ��γ��
//							if(gpsStr)	//�����ȡ����
//							{
//								strcat(gpsDatalat,gpsStr);//����ά��
//							}
//							else	//��û��λ�� ��Ĭ�Ϸ�������λ��
//							{
//								strcat(gpsDatalat,"39.897445");//����ά��
//							}
//							
//							gpsStr=Get_GPS_RMC(2);//��ȡά�� ����Ǿ�����ƫ�ľ���
//							if(gpsStr)					  //��ȡ���˾���
//							{
//								strcat(gpsDatalon,gpsStr);//���ݾ���
//							}
//							else	//��û��λ�� ��Ĭ�Ϸ�������λ��
//							{
//								strcat(gpsDatalon,"116.331398");//���ݾ���
//							}
//							
//							Get_4GIMEI_NUM();  //��ȡIMEI�������к�
//							SendData_aliyunStudio("longitude",gpsDatalon);//���;�������
//							SendData_aliyunStudio("latitude",gpsDatalat);//����γ������




