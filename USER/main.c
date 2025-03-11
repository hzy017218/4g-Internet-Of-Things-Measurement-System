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
///////////////下面是液晶屏头文件/////////////////////
#include "Lcd_Driver.h"
#include "GUI.h"
#include "delay.h"
#include "Picture.h"
#include "QDTFT_demo.h"
#include "dht11.h"
/////////////////////////////////////////////////////
//主程序开始
//发送两路传感器 温湿度 和GPS北斗定位信息
////////////////////////////////////////////////////
int main(void)
{
	char strValue[12]; // 用于存储字符串，足够容纳最大值和结束符
	char temp_disp[BUFLEN];	//温度显示
	char dht[8];
	int last_temp1 = 0;
    int last_temp2 = 0;
	TemperatureData temps;
	
//	u16 adcx1,adcx2; //ADC1和ADC2数值
//	char adcValue[8];
	
	
	/**********************************初始化************************************************************/
	
  delay_init();	    	 //延时函数初始化	  
  NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	CSTX_4GCTR_Init();        //初始化CSTX_4G的供电引脚 对模块进行供电
  uart_init(115200);//串口1初始化，可连接PC进行打印模块返回数据
	Uart1_SendStr("UART1 Init Successful\r\n");
	uart2_init(115200);//初始化和EC200连接串口	
	Uart2_SendStr("UART2 Init Successful\r\n");
	uart3_init(115200);
	Uart3_SendStr("UART3 Init Successful\r\n");
	RS485_Init(9600);
	RS485_SendStr("RS485 Init Successful\r\n");
	LED_Init();		  		//初始化与LED连接的硬件接口
	
	//////////下面是液晶屏显示代码///////////////////////////
	Lcd_Init();
	LCD_LED_SET;//通过IO控制背光亮				
	Num_Test();//数码管字体测试	
	showimage(gImage_qq);//图片显示示例
	Font_Test();//中英文显示测试	
	delay_ms(1200);
	/////////////////////////////////////////////////////////////   
	LED_Run();					//初始化跑马灯 
	Lcd_Clear(WHITE);
	Close_MQTT_aliyunStudio();//关闭上一次连接
	CSTX_4G_Init();//对设备初始化
	Connect_MQTT_aliyunStudio();//创建一个连接
	Start_GPS();//早点开启GPS让他定位好
	Clear_Buffer_UART1();
	Display_Menu();
	DHT11_Init();	//初始化温湿度 用PA11
	Adc_Init();	  	//ADC初始化
	TIM3_Int_Init(4999,7199);//100ms触发一次  
	PT100_init();
	
	
//	/**********************************开机发送************************************************************/
//	Get_PT100_data();
//	delay_ms(1200);
//	PT100_data();
//	memset(dht,0,8);
//	sprintf(dht,"%d",data_t);
//	SendData_aliyunStudio("temp",(char *)dht);//发送温度数据
//	
//	memset(temp_disp,0,BUFLEN);
//	sprintf(temp_disp,"temp1:%d,temp2:%d",data_t,DHT11_Data.humi_int);	//温湿度打印到数组
//	Gui_DrawFont_GBK16(0,50,RED,WHITE, (u8*)temp_disp); //温湿度显示到液晶屏	
	
	while(1)
	{  
	Control_Led();     //远程重启

	Get_PT100_data();
	delay_ms(500);  
//			PT100_data(); 
	temps = PT100_data();	  

	/**********************************实时发送************************************************************/

	/***********************高度实时发送******************************/

	if ((value_shiji - last_value >= 1) || (value_shiji - last_value <= -1)) 
	{
		// 检查value_shiji 是否溢出
		if (value_shiji > 65535) {
				value_shiji = 65535;  
		} 
		sprintf(strValue, "%u", value_shiji);
		SendData_aliyunStudio("485", (char *)strValue);
		
		printf("Parsed Data_Height: %u\n", value_shiji);
		last_value = value_shiji; 
		
		memset(strValue,0,BUFLEN);
		sprintf(strValue,"Height:%d",value_shiji);	//高度打印到数组
		Gui_DrawFont_GBK16(10,90,RED,WHITE,(u8*)strValue);
	}
			
		/***********************温度实时发送***************************/
	
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
//				SendData_aliyunStudio("temp",(char *)dht);//发送温度数据
//				printf("Parsed Data————Temp1: %d\n", data_t);  
//				last_temp = data_t;

//				memset(temp_disp, 0, BUFLEN);
//				sprintf(temp_disp, "Temp1:%d", data_t);
//				Gui_DrawFont_GBK16(0, 110, RED, WHITE, (u8*)temp_disp);
//			}

				
				
	/*********************************定时发送************************************************************/
	
	if(time3_5s >= 7200)	//定时器 60    秒钟发送一次数据
	{							
			/**********************高度定时发送****************************/
		
			sprintf(strValue, "%u", value_shiji);
			SendData_aliyunStudio("485", (char *)strValue);
			printf("Parsed Data: %u\n", value_shiji);
			last_value = value_shiji; 
			
			memset(strValue,0,BUFLEN);
			sprintf(strValue,"Height:%d",value_shiji);	//高度打印到数组
			Gui_DrawFont_GBK16(10,90,RED,WHITE,(u8*)strValue);
			////////////////////////////////////////////////////////////////

			/**********************温度定时发送******************************/
//							DHT11_Read_TempAndHumidity();
			Get_PT100_data();
			delay_ms(200);
			temps = PT100_data();

			memset(dht,0,8);
			sprintf(dht,"%d",temps.temperature1);
			SendData_aliyunStudio("temp",(char *)dht);//发送温度1

			memset(dht,0,8);
			sprintf(dht,"%d",temps.temperature2);
			SendData_aliyunStudio("humi",(char *)dht);//发送温度2
								
			memset(temp_disp,0,BUFLEN);
//			sprintf(temp_disp,"Temp1:%d Temp2:%d",temps.temperature1,temps.temperature2);	//温湿度打印到数组
			sprintf(temp_disp,"Temp1:%d",temps.temperature1);	//温湿度打印到数组
			Gui_DrawFont_GBK16(10,110,RED,WHITE, (u8*)temp_disp); //温湿度显示到液晶屏	

			memset(temp_disp,0,BUFLEN);
			sprintf(temp_disp,"Temp2:%d",temps.temperature2);
			Gui_DrawFont_GBK16(10,130,RED,WHITE, (u8*)temp_disp); //温湿度显示到液晶屏	
			////////////////////////////////////////////////////////////////
									

			LED1=!LED1; //系统运行指示灯
		}
  }	 
}

						


//							//发送adc1 adc2 //////////////////////////////////////////////////////////////////////
//							adcx1=Get_Adc_Average(ADC_Channel_10,10); //获取得到PC0的ADC1的值
//							adcx2=Get_Adc_Average(ADC_Channel_11,10);
//							printf("ADC1原始数值：%d, ADC2原始数值：%d\r\n",(int)((float)adcx1/4096*100),(int)((float)adcx2/4096*100)); //打印原始采集的数据
//							memset(dht,0,8);	//清空传感器1数组
//							sprintf(adcValue,"%d",(int)((float)adcx1/4096*100));
//							SendData_aliyunStudio("ADC1",adcValue);
//							memset(dht,0,8);	//清空传感器2数组
//							sprintf(adcValue,"%d",(int)((float)adcx2/4096*100));
//							SendData_aliyunStudio("ADC2",adcValue);	
//							memset(temp_disp,0,BUFLEN);
//							sprintf(temp_disp,"ADC1:%2d,ADC2:%2d",(int)((float)adcx1/4096*100),(int)((float)adcx2/4096*100));	//温湿度打印到数组
//							Gui_DrawFont_GBK16(0,130,BLACK,WHITE, (u8*)temp_disp); //温湿度显示到液晶屏			
							//////////////////////////////////////////////////////////////////////////////////////////



