#include "4G.h"
#include "string.h"
#include "usart.h"
#include "wdg.h"
#include "led.h"
#include "rs485.h"
#include <math.h>
///////////////������Һ����ͷ�ļ�/////////////////////
#include "Lcd_Driver.h"
#include "GUI.h"
#include "delay.h"
//#include "Picture.h"
#include "QDTFT_demo.h"
#include <stdlib.h> 
#include <math.h>
/////////////////////////////////////////////////////
char *strx,*extstrx;

CSTX_4G CSTX_4G_Status;	//ģ���״̬��Ϣ
int  errcount=0;	//ʧ�ܴ��� ��ֹ��ѭ��
char ATSTR[BUFLEN];	//�齨AT����ĺ���
char GPRMCSTR[128]; //ת��GPS��Ϣ GPRMC ��γ�ȴ洢���ַ���
char GPRMCSTRLON[64]; //���ȴ洢�ַ��� Ҳ����119.20694
char GPRMCSTRLAT[64]; //ά�ȴ洢�ַ�����Ҳ����26.06451
char IMEINUMBER[64];//+CGSN: "869523052178994"


const double PI = 3.14159265358979324;
const double a = 6378245.0;
const double ee = 0.00669342162296594323;
const double x_pi = 3.14159265358979324 * 3000.0 / 180.0;

double lng_r,lat_r,tran_lng,tran_lat;
double dlat,dlng,radlat,magic,sqrtmagic;
double mglat,mglng,lngtitude,lattitude;

//////////////////�����Ǿ�����������ı�������/////////////////////////
int Get_GPSdata(void);
void Getdata_Change(char status);

 typedef struct 
{
char UtcDate[6];
char longitude[11];//����ԭ����
char Latitude[10];//γ��Դ����
char longitudess[4];//��������
char Latitudess[3];
char longitudedd[8];//С���㲿��
char Latitudedd[8];
char Truelongitude[12];//ת��������
char TrueLatitude[11];//ת��������
	
	
char getstautus;//��ȡ����λ�ı�־״̬	
float gpsdata[2];
}LongLatidata;
LongLatidata latdata;

float tempdata[2];
char latStrAF[64];          //������ݾ�γ����������
char lonStrAF[64];   //������ݾ�γ��������ʾ
/*****************************************************
���������Ҫ�޸ĵĵط����޸ķ�������IP��ַ�Ͷ˿ں�
*****************************************************/
//#define ServerIP		"k1p4tp3OQ5Y.iot-as-mqtt.cn-shanghai.aliyuncs.com" //���� ע��Ҫ�޸Ĳ�Ʒid
#define ServerIP		"iot-06z00a5nclva5b8.mqtt.iothub.aliyuncs.com" //���� ע��Ҫ�޸Ĳ�Ʒid
#define Port 				"1883" //mqtt�˿�

#define ClientID 		"k2602kzAbjB.EC200U|securemode=2,signmethod=hmacsha256,timestamp=1734939913188|"    //��Ҫ����Ϊ�û��Լ��Ĳ��� �ͻ���id clientId
#define Username 		"EC200U&k2602kzAbjB"		//��Ҫ����Ϊ�û��Լ��Ĳ��� �û���
#define Password 		"03b3fbe12701de13cec4789d7a111b0a18af9679a256ed684346d118fe1101c5"  //����

#define TopicPost   "/sys/k2602kzAbjB/EC200U/thing/event/property/post"  //�޸Ĳ�Ʒid���豸����
#define Topic   		"/sys/k2602kzAbjB/EC200U/thing/service/property/set" //�޸Ĳ�Ʒid���豸����

/*****************************************************
���ģ�鷴������Ϣ
*****************************************************/


void Clear_Buffer(void)//��ջ���
{
		printf(buf_uart2.buf);
		Control_Led();
    delay_ms(10);
    buf_uart2.index=0;
	
    memset(buf_uart2.buf,0,BUFLEN);
}


/*****************************************************
��ʼ��ģ�� �͵�Ƭ�����ӣ���ȡ���ź��ź�����
*****************************************************/
void CSTX_4G_Init(void)
{
	//��ӡ��ʼ����Ϣ
	printf("start init EC200U\r\n");
	//����һ������ATE1
	Uart2_SendStr("ATE1\r\n"); 
	delay_ms(300);
	printf(buf_uart2.buf);      //��ӡ�����յ�����Ϣ
	strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//����OK
	Clear_Buffer();	
	while(strx==NULL)
	{
			//Gui_DrawFont_GBK16(16,70,RED,WHITE, "ȫ�̼���֧��");
			Gui_DrawFont_GBK16(16,10,RED,WHITE, "CONNECT 4G..");
			printf("��Ƭ����������ģ��......\r\n");
			Clear_Buffer();	
			Uart2_SendStr("ATE1\r\n"); 
			delay_ms(300);
			strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//����OK
	}
	Gui_DrawFont_GBK16(16,10,RED,WHITE, "CONNECT [OK]");
	printf("****��Ƭ����ģ�����ӳɹ�*****\r\n");
	Uart2_SendStr("ATI\r\n");//��ȡģ��İ汾
	delay_ms(300);
	Clear_Buffer();	
	
	Uart2_SendStr("AT+QCCID\r\n");//��ȡ�������к� 
	delay_ms(300);
	Clear_Buffer();	
	
	Uart2_SendStr("AT+CGSN\r\n");//��ȡģ���imei
	delay_ms(300);
	Clear_Buffer();	
	
	Uart2_SendStr("AT+CIMI\r\n");//��ȡ���ţ������Ƿ���ڿ�����˼���Ƚ���Ҫ��
	delay_ms(300);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"460");//��460������ʶ�𵽿���
	while(strx==NULL)
	{
			Clear_Buffer();	
			Uart2_SendStr("AT+CIMI\r\n");//��ȡ���ţ������Ƿ���ڿ�����˼���Ƚ���Ҫ��
			delay_ms(300);
			strx=strstr((const char*)buf_uart2.buf,(const char*)"460");//����OK,˵�����Ǵ��ڵ�
	} 
	printf("�ҵĿ����� : %s \r\n",buf_uart2.buf+8);
	Clear_Buffer();	
	
	Gui_DrawFont_GBK16(16,10,RED,WHITE, "SIMCARD [OK]");
	
	Uart2_SendStr("AT+CGATT?\r\n");//��ѯ����״̬
	delay_ms(300);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"+CGATT: 1");//��1
	Clear_Buffer();	
	while(strx==NULL)
	{
			Clear_Buffer();	
			Uart2_SendStr("AT+CGATT?\r\n");//��ȡ����״̬
			delay_ms(300);
			strx=strstr((const char*)buf_uart2.buf,(const char*)"+CGATT: 1");//����1,����ע���ɹ�
	}
	
	Gui_DrawFont_GBK16(16,10,RED,WHITE, "REGISTER[OK]");
	
	
	Clear_Buffer();	
	Uart2_SendStr("AT+CSQ\r\n");//�鿴��ȡCSQֵ
	delay_ms(300);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"+CSQ:");//����CSQ
	if(strx)
	{
			printf("�ź�������:%s ע�⣺�ź����ֵ��31 \r\n",buf_uart2.buf+14);      
	}
		
}


char* Get_4GIMEI_NUM(void)
{
		Clear_Buffer();	
		memset(IMEINUMBER,0,64);
		Uart2_SendStr("AT+CGSN=1\r\n");//��ѯ����״̬
		delay_ms(100);
		strx=strstr((const char*)buf_uart2.buf,(const char*)"+CGSN: \"");//�����������ͱ�ʾû�ж�λ��
		if(strx)	//û�з�������ͱ�ʾ�о�γ���� Ȼ����������ʾ �����õ�LOC�ͱ�ʾ��λ����
		{
				strncpy(IMEINUMBER,strx+8,15); //��ȡά������
				Gui_DrawFont_GBK16(0,70,PINNK,WHITE, (u8*) IMEINUMBER);	//��ʾ���ȵ�Һ����
				return IMEINUMBER;
		}
		Clear_Buffer();	
		return 0;
}
/*
AT+QGPSLOC=0

+QGPSLOC: 035658.000,2603.8722N,11912.4159E,1.8,269.9,3,000.00,0.4,0.2,140821,10

OK
AT+QGPSLOC=1

+QGPSLOC: 035702.000,2603.871651,N,11912.416382,E,1.8,268.8,3,000.00,1.2,0.6,140821,11

OK
AT+QGPSLOC=2

+QGPSLOC: 035704.000,26.06451,119.20694,1.8,270.8,3,000.00,0.2,0.1,140821,11

OK
*/
char* Get_GPS_LOC(char type)
{
		Clear_Buffer();	
		memset(GPRMCSTR,0,128);
		Uart2_SendStr("AT+QGPSLOC=0\r\n");//��ѯ����״̬
		delay_ms(300);
		strx=strstr((const char*)buf_uart2.buf,(const char*)"+QGPSLOC:");//�����������ͱ�ʾû�ж�λ��
		
		if(strx)	//û�з�������ͱ�ʾ�о�γ���� Ȼ����������ʾ �����õ�LOC�ͱ�ʾ��λ����
		{
				//��ȡ����γ��
				strncpy(GPRMCSTR,strx+21,22);
				printf("ģ�鶨λ���˾�γ���� %s\r\n",GPRMCSTR); //��γ���ַ���ֱ�Ӵ�ӡ������
				//+QGPSLOC: 035658.000,2603.8722N,11912.4159E,1.8,269.9,3,000.00,0.4,0.2,140821,10
				//ά��26.06451
				memset(GPRMCSTRLAT,0,64);
				strncpy(GPRMCSTRLAT,strx+21,2); //��ȡά������
				strcat(GPRMCSTRLAT,".");
				strncpy(GPRMCSTRLAT+strlen((char*)GPRMCSTRLAT),strx+23,2); //��ȡά������
				strncpy(GPRMCSTRLAT+strlen((char*)GPRMCSTRLAT),strx+26,4); //��ȡά������
			
				//Gui_DrawFont_GBK16(0,90,RED,WHITE, (u8*) GPRMCSTRLAT);	//��ʾ���ȵ�Һ����
				//���� 119.20694
				memset(GPRMCSTRLON,0,64);
				strncpy(GPRMCSTRLON,strx+32,3);	//��ȡ����
				strcat(GPRMCSTRLON,".");
				strncpy(GPRMCSTRLON+strlen((char*)GPRMCSTRLON),strx+35,2);	//��ȡ����
				strncpy(GPRMCSTRLON+strlen((char*)GPRMCSTRLON),strx+38,4);	//��ȡ����
				//Gui_DrawFont_GBK16(0,110,RED,WHITE, (u8*) GPRMCSTRLON);	//��ʾ���ȵ�Һ����
				Clear_Buffer();	
				if(type==1)
				{
					return GPRMCSTRLAT;
				}
				if(type==2)
				{
					return GPRMCSTRLON;
				}
				
		}
		return 0;
}

/*
AT+QGPSGNMEA="RMC"

+QGPSGNMEA: $GNRMC,035645.00,A,2603.9111,N,11912.4140,E,0.336,,140821,,,A,V*19

OK
*/

char *Get_GPS_RMC(char type)
{
		Clear_Buffer();	
		memset(GPRMCSTR,0,128);
		Uart2_SendStr("AT+QGPSGNMEA=\"RMC\"\r\n");//��ѯ����״̬
		delay_ms(100);
		strx=strstr((const char*)buf_uart2.buf,(const char*)"$GNRMC");//��1
		errcount=0;	
		while(strx==NULL)
		{
				Clear_Buffer();	
				Uart2_SendStr("AT+QGPSGNMEA=\"RMC\"\r\n");//��ȡ����״̬
				delay_ms(30);
				strx=strstr((const char*)buf_uart2.buf,(const char*)"$GNRMC");//����1,����ע���ɹ�
				errcount++;
				if(	errcount > 200)
				{
					errcount=0;
					GPIO_SetBits(GPIOA,GPIO_Pin_12); 	
					delay_ms(1000);
					GPIO_ResetBits(GPIOA,GPIO_Pin_12); 	 
					delay_ms(300);
					NVIC_SystemReset();	//һֱû�л�ȡ��GPS��λ�͸�λ��
				}
		}
		sprintf(GPRMCSTR,"%s",strx);
		//Gui_DrawFont_GBK16(16,10,RED,WHITE, "STARTGPS[OK]");
		//Gui_DrawFont_GBK16(16,10,GREEN,WHITE, "GET  GPS INFO");
		Gui_DrawFont_GBK16(16,10,GREEN,WHITE, "GPS INFO [..]");
		Clear_Buffer();																							    //��ӡ�յ���GPS��Ϣ
		GPRMCSTR[2]=	'P';
		
		printf("============GETGPRMC==============\r\n%s",GPRMCSTR);		//��ӡGPRMC
		if(GPRMCSTR[17]=='A')
		{
			memset(latStrAF,0,64);
			memset(lonStrAF,0,64);
			Get_GPSdata();
			Gui_DrawFont_GBK16(0,90,BLUE,WHITE, (u8*) latStrAF);	//��ʾ���ȵ�Һ����
			Gui_DrawFont_GBK16(0,110,BLUE,WHITE, (u8*) lonStrAF);	//��ʾ���ȵ�Һ����
			Gui_DrawFont_GBK16(16,10,BLUE,WHITE, "GPS INFO [OK]");
			if(type==1)
				return latStrAF;
			if(type==2)
				return lonStrAF;
		}
		return 0;
}



/*****************************************************
�����ǽ������������
*****************************************************/


//��GPS������
//$GPRMC,134952.00,A,2603.9576,N,11912.4098,E,0.154,,280821,,,A,V*18
int Get_GPSdata()
{
		int i=0;
    strx=strstr((const char*)GPRMCSTR,(const char*)"A,");//��ȡγ�ȵ�λ��
       if(strx)
        {
            for(i=0;i<9;i++)
            {
             latdata.Latitude[i]=strx[i+2];//��ȡγ��ֵ2603.9576
            }
						strx=strstr((const char*)GPRMCSTR,(const char*)"N,");//��ȡ����ֵ
						if(strx)
						{
								 for(i=0;i<10;i++)	//��ȡ���� 11912.4098
								 {
										latdata.longitude[i]=strx[i+2];
								 }
								 
						}  
						
						printf("latdata.Latitude ,%s \r\n",latdata.Latitude);
						printf("latdata.longitude ,%s \r\n",latdata.longitude);
            latdata.getstautus=1;//                 
	    }
                            
		else
		{
						
				latdata.getstautus=0;
		 }
			Getdata_Change(latdata.getstautus);//���ݻ���
			Clear_Buffer();
		 return 0;

}





/*************��������γ������,Ȼ��ֱ���ύ����*******************/	

void Getdata_Change(char status)
{
	unsigned char i;	
    	
    if(status)
    {

        for(i=0;i<3;i++)
						latdata.longitudess[i]=latdata.longitude[i];
				for(i=3;i<10;i++)
						latdata.longitudedd[i-3]=latdata.longitude[i];
			
			 latdata.gpsdata[0]=(latdata.longitudess[0]-0x30)*100+(latdata.longitudess[1]-0x30)*10+(latdata.longitudess[2]-0x30)\
		     +((latdata.longitudedd[0]-0x30)*10+(latdata.longitudedd[1]-0x30)+(float)(latdata.longitudedd[3]-0x30)/10+\
		     (float)(latdata.longitudedd[4]-0x30)/100+(float)(latdata.longitudedd[5]-0x30)/1000+(float)(latdata.longitudedd[6]-0x30)/10000)/60.0;//��ȡ����������
       
///////////////////////////////////////////
				for(i=0;i<2;i++)
						latdata.Latitudess[i]=latdata.Latitude[i];
				for(i=2;i<9;i++)
						latdata.Latitudedd[i-2]=latdata.Latitude[i];	
				 
			latdata.gpsdata[1]=(float)(latdata.Latitudess[0]-0x30)*10+(latdata.Latitudess[1]-0x30)\
		     +((latdata.Latitudedd[0]-0x30)*10+(latdata.Latitudedd[1]-0x30)+(float)(latdata.Latitudedd[3]-0x30)/10+\
		     (float)(latdata.Latitudedd[4]-0x30)/100+(float)(latdata.Latitudedd[5]-0x30)/1000+(float)(latdata.Latitudedd[6]-0x30)/10000)/60.0;//��ȡ����������b

	
				 sprintf(latStrAF,"%f",latdata.gpsdata[1]);
				 sprintf(lonStrAF,"%f",latdata.gpsdata[0]);
				 
				 			 
				 printf("latStrAF,%s \r\n",latStrAF);
				 printf("lonStrAF,%s \r\n",lonStrAF);
				 
    }
    else
    {
        latdata.gpsdata[0]=0;
        latdata.gpsdata[1]=0;
    }
		
	
}
void WG_to_GCJ(float lng,float lat)
{
	lng_r = lng - 105.0;
	lat_r = lat - 35.0;
	//???? 
	tran_lng= 300.0 + lng_r + 2.0 * lat_r + 0.1 * lng_r * lng_r + 0.1 * lng_r * lat_r + 0.1 * sqrt(abs(lng_r));
	tran_lng = tran_lng + (20.0 * sin(6.0 * lng_r * PI) + 20.0 * sin(2.0 * lng_r * PI)) * 2.0 / 3.0;
	tran_lng = tran_lng + (20.0 * sin(lng_r * PI) + 40.0 * sin(lng_r / 3.0 * PI)) * 2.0 / 3.0;
	tran_lng = tran_lng + (150.0 * sin(lng_r / 12.0 * PI) + 300.0 * sin(lng_r / 30.0 * PI)) * 2.0 / 3.0;
	//????
	tran_lat = -100.0 + 2.0 * lng_r + 3.0 * lat_r + 0.2 * lat_r * lat_r + 0.1 * lng_r * lat_r + 0.2 * sqrt(abs(lng_r));
	tran_lat = tran_lat+(20.0 * sin(6.0 * lng_r * PI) + 20.0 * sin(2.0 * lng_r * PI)) * 2.0 / 3.0;
	tran_lat = tran_lat+(20.0 * sin(lat_r * PI) + 40.0 * sin(lat_r / 3.0 * PI)) * 2.0 / 3.0;
	tran_lat = tran_lat+(160.0 * sin(lat_r / 12.0 * PI) + 320 * sin(lat_r * PI / 30.0)) * 2.0 / 3.0;
	
	dlat = tran_lat;
	dlng = tran_lng;
	
	radlat = lat/180.0 * PI;
	magic = sin(radlat);
	magic = 1- ee * magic * magic;
	sqrtmagic = sqrt(magic);
	dlat = (dlat * 180.0)/((a * (1-ee))/(magic * sqrtmagic)*PI);
	dlng = (dlng * 180.0)/(a / sqrtmagic * cos(radlat) * PI);
	mglat = lat + dlat;
	mglng = lng + dlng;
	printf("%lf,%lf\r\n",mglat,mglng);
 } 


/*****************************************************
��ƫ����
*****************************************************/

void Start_GPS(void)
{
		Clear_Buffer();	
		Uart2_SendStr("AT+QGPS=1\r\n");//��ѯ����״̬
		delay_ms(300);
		strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//��1
		
		while(strx==NULL)
		{
				delay_ms(300);
				strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//����1,����ע���ɹ�
		}
		
		Gui_DrawFont_GBK16(16,10,RED,WHITE, "STARTGPS[OK]");
		Clear_Buffer();	
}

//�ر���OneNet Studioƽ̨������
void Close_MQTT_aliyunStudio(void)
{
	//�ر�֮ǰ����������
	Uart2_SendStr("AT+QMTDISC=0\r\n");
	delay_ms(100);
	Clear_Buffer();
} 

//������Onenet Studio������
void Connect_MQTT_aliyunStudio(void)
{
	printf("\r\n������Onenet Studio������\r\n");
	Uart2_SendStr("AT+QMTCFG=\"recv/mode\",0,0,1\r\n");//���ý���ģʽ
	delay_ms(300);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//��OK
	Clear_Buffer();	
	while(strx==NULL)
	{
			Clear_Buffer();	
			Uart2_SendStr("AT+QMTCFG=\"recv/mode\",0,0,1\r\n");//���ý���ģʽ
			delay_ms(300);
			strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//����OK,�����ɹ�
	}
	Clear_Buffer();
	
	Uart2_SendStr("AT+QMTCFG=\"version\",0,4\r\n");//���� MQTT Э��汾
	delay_ms(300);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//��OK
	Clear_Buffer();	
	while(strx==NULL)
	{
			Clear_Buffer();	
			Uart2_SendStr("AT+QMTCFG=\"version\",0,4\r\n");//���� MQTT Э��汾
			delay_ms(300);
			strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//����OK,�����ɹ�
	}
	Clear_Buffer();
	
	memset(ATSTR,0,BUFLEN);
	sprintf(ATSTR,"AT+QMTOPEN=0,\"%s\",%s\r\n",ServerIP,Port); 
	Uart2_SendStr(ATSTR);//��MQTT�ͻ�������
	delay_ms(300);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//��OK
	errcount=0;	
	while(strx==NULL)
	{
		errcount++;
		strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//����OK
		delay_ms(100);
		if(errcount>100)     //��ʱ�˳���ѭ�� ��ʾ����������ʧ��
		{
			errcount = 0;
			break;
		}
	}
	Clear_Buffer();
	
	Gui_DrawFont_GBK16(16,10,RED,WHITE, "  MQTT  [OK]");
	
	memset(ATSTR,0,BUFLEN);
	sprintf(ATSTR,"AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"\r\n",ClientID,Username,Password); 
	
	Uart2_SendStr(ATSTR);//���ӿͻ��˵�MQTT������
	delay_ms(300);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"+QMTCONN: 0,0,0");//+QMTCONN: 0,0,0
	errcount=0;	
	while(strx==NULL)
	{
		errcount++;
		strx=strstr((const char*)buf_uart2.buf,(const char*)"+QMTCONN: 0,0,0");//����+QMTCONN: 0,0,0
		Gui_DrawFont_GBK16(0,50,RED,WHITE, (u8*)strx);
		delay_ms(100);
		if(errcount>100)     //��ʱ�˳���ѭ�� ��ʾ����������ʧ��
		{
			Gui_DrawFont_GBK16(16,10,RED,WHITE, "  MQTT  [NO]");
			errcount = 0;
			break;
		}
	}
	Clear_Buffer();
	printf("\r\n���ӳɹ�\r\n");
	Gui_DrawFont_GBK16(16,10,RED,WHITE, "ALICONNECTOK");
	
	memset(ATSTR,0,BUFLEN);
	sprintf(ATSTR,"AT+QMTSUB=0,1,\"%s\",2\r\n",Topic); 
	Uart2_SendStr(ATSTR);//����
	delay_ms(300);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"+QMTSUB: 0,1,0,1");//+QMTCONN: 0,0,0
	errcount=0;	
	while(strx==NULL)
	{
		errcount++;
		strx=strstr((const char*)buf_uart2.buf,(const char*)"+QMTSUB: 0,1,0,1");//����+QMTCONN: 0,0,0
		delay_ms(30);
		if(errcount>100)     //��ʱ�˳���ѭ�� ��ʾ����ʧ��
		{
			Gui_DrawFont_GBK16(10,50,RED,WHITE, "subscribe NO");
			errcount = 0;
			break;
		}
	}
	Gui_DrawFont_GBK16(10,50,RED,WHITE, "subscribe OK");
	Clear_Buffer();
	printf("\r\n���ĳɹ�\r\n");
}

//�������ݵ�OneNet Studio
void SendData_aliyunStudio(char *ID,char *Data)
{
	int len = 0;
	char Buff[200]={0};
	Gui_DrawFont_GBK16(16,10,RED,WHITE, "Send DATA[..]");
	memset(Buff,0,sizeof(Buff));
	sprintf(Buff,"{\"id\":\"1687956949030\",\"version\":\"1.0\",\"params\":{\"%s\":{\"value\":%s}},\"method\":\"thing.event.property.post\"}",ID,Data); 
	len = strlen(Buff); 
	memset(ATSTR,0,BUFLEN);
	sprintf(ATSTR,"AT+QMTPUBEX=0,0,0,0,\"%s\",%d\r\n",TopicPost,len);
	Uart2_SendStr(ATSTR);//������Ϣ
	delay_ms(100);
	strx=strstr((const char*)buf_uart2.buf,(const char*)">");
	errcount=0;	
	while(strx==NULL)
	{
		errcount++;
		strx=strstr((const char*)buf_uart2.buf,(const char*)">");
		delay_ms(30);
		if(errcount>100)
		{
			errcount = 0;
			break;
		}
	}
	Clear_Buffer();
	printf("\r\n���͵�JSON����:%s\r\n",Buff);
	Uart2_SendStr(Buff);//��������
	delay_ms(100);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"+QMTPUBEX: 0,0,0");
	if(strx!=NULL)
	{
		printf("\r\n���ݷ��ͳɹ�\r\n");
	}
	Gui_DrawFont_GBK16(16,10,BLUE,WHITE, "Send DATA[OK]");
}


void Control_Led(void)
{

		strx=strstr((const char*)buf_uart2.buf,(const char*)"\"btn1\":1");//����+QMTRECV:���������յ����������ص����� ����1
    if(strx)
		{
			Gui_DrawFont_GBK16(16,10,RED,WHITE, "REBOOTING STM32");  
//			LED1=0;
			NVIC_SystemReset();
			buf_uart2.index=0;
			memset(buf_uart2.buf,0,BUFLEN);
			return ;
		}
		strx=strstr((const char*)buf_uart2.buf,(const char*)"\"btn1\":0");//����+QMTRECV:���������յ����������ص����� �رյ�1
    if(strx)
		{
			Gui_DrawFont_GBK16(16,10,RED,WHITE, "TURNOFF LED1");  
//			LED1=1;
			buf_uart2.index=0;
			memset(buf_uart2.buf,0,BUFLEN);
			return ;
		}
		strx=strstr((const char*)buf_uart2.buf,(const char*)"\"btn2\":1");//����+QMTRECV:���������յ����������ص����� ����2 
    if(strx)
		{
			Gui_DrawFont_GBK16(16,10,RED,WHITE, "TURN ON LED2");  
			LED2=0;
			buf_uart2.index=0;
			memset(buf_uart2.buf,0,BUFLEN);
			return ;
		}
		strx=strstr((const char*)buf_uart2.buf,(const char*)"\"btn2\":0");//����+QMTRECV:���������յ����������ص����� �رյ�2 
    if(strx)
		{
			Gui_DrawFont_GBK16(16,10,RED,WHITE, "TURNOFF LED2");  
			LED2=1;
			buf_uart2.index=0;
      memset(buf_uart2.buf,0,BUFLEN);
			return ;
		}
		strx=strstr((const char*)buf_uart2.buf,(const char*)"\"btn3\":1");//����+QMTRECV:���������յ����������ص����� ����3 
    if(strx)
		{
			Gui_DrawFont_GBK16(16,10,RED,WHITE, "TURN ON LED3");  
			LED3=0;
			buf_uart2.index=0;
      memset(buf_uart2.buf,0,BUFLEN);
			return ;
		}
		strx=strstr((const char*)buf_uart2.buf,(const char*)"\"btn3\":0");//����+QMTRECV:���������յ����������ص����� �رյ�3
    if(strx)
		{
			Gui_DrawFont_GBK16(16,10,RED,WHITE, "TURNOFF LED3");  
			LED3=1;
			buf_uart2.index=0;
      memset(buf_uart2.buf,0,BUFLEN);
			return ;
		}
}









