#include "4G.h"
#include "string.h"
#include "usart.h"
#include "wdg.h"
#include "led.h"
#include "rs485.h"
#include <math.h>
///////////////下面是液晶屏头文件/////////////////////
#include "Lcd_Driver.h"
#include "GUI.h"
#include "delay.h"
//#include "Picture.h"
#include "QDTFT_demo.h"
#include <stdlib.h> 
#include <math.h>
/////////////////////////////////////////////////////
char *strx,*extstrx;

CSTX_4G CSTX_4G_Status;	//模块的状态信息
int  errcount=0;	//失败次数 防止死循环
char ATSTR[BUFLEN];	//组建AT命令的函数
char GPRMCSTR[128]; //转载GPS信息 GPRMC 经纬度存储的字符串
char GPRMCSTRLON[64]; //经度存储字符串 也就是119.20694
char GPRMCSTRLAT[64]; //维度存储字符串，也就是26.06451
char IMEINUMBER[64];//+CGSN: "869523052178994"


const double PI = 3.14159265358979324;
const double a = 6378245.0;
const double ee = 0.00669342162296594323;
const double x_pi = 3.14159265358979324 * 3000.0 / 180.0;

double lng_r,lat_r,tran_lng,tran_lat;
double dlat,dlng,radlat,magic,sqrtmagic;
double mglat,mglng,lngtitude,lattitude;

//////////////////下面是纠正火星坐标的变量定义/////////////////////////
int Get_GPSdata(void);
void Getdata_Change(char status);

 typedef struct 
{
char UtcDate[6];
char longitude[11];//经度原数据
char Latitude[10];//纬度源数据
char longitudess[4];//整数部分
char Latitudess[3];
char longitudedd[8];//小数点部分
char Latitudedd[8];
char Truelongitude[12];//转换过数据
char TrueLatitude[11];//转换过数据
	
	
char getstautus;//获取到定位的标志状态	
float gpsdata[2];
}LongLatidata;
LongLatidata latdata;

float tempdata[2];
char latStrAF[64];          //存放数据经纬度用来发送
char lonStrAF[64];   //存放数据经纬度用来显示
/*****************************************************
下面就是需要修改的地方，修改服务器的IP地址和端口号
*****************************************************/
//#define ServerIP		"k1p4tp3OQ5Y.iot-as-mqtt.cn-shanghai.aliyuncs.com" //域名 注意要修改产品id
#define ServerIP		"iot-06z00a5nclva5b8.mqtt.iothub.aliyuncs.com" //域名 注意要修改产品id
#define Port 				"1883" //mqtt端口

#define ClientID 		"k2602kzAbjB.EC200U|securemode=2,signmethod=hmacsha256,timestamp=1734939913188|"    //需要定义为用户自己的参数 客户端id clientId
#define Username 		"EC200U&k2602kzAbjB"		//需要定义为用户自己的参数 用户名
#define Password 		"03b3fbe12701de13cec4789d7a111b0a18af9679a256ed684346d118fe1101c5"  //密码

#define TopicPost   "/sys/k2602kzAbjB/EC200U/thing/event/property/post"  //修改产品id和设备名称
#define Topic   		"/sys/k2602kzAbjB/EC200U/thing/service/property/set" //修改产品id和设备名称

/*****************************************************
清空模块反馈的信息
*****************************************************/


void Clear_Buffer(void)//清空缓存
{
		printf(buf_uart2.buf);
		Control_Led();
    delay_ms(10);
    buf_uart2.index=0;
	
    memset(buf_uart2.buf,0,BUFLEN);
}


/*****************************************************
初始化模块 和单片机连接，获取卡号和信号质量
*****************************************************/
void CSTX_4G_Init(void)
{
	//打印初始化信息
	printf("start init EC200U\r\n");
	//发第一个命令ATE1
	Uart2_SendStr("ATE1\r\n"); 
	delay_ms(300);
	printf(buf_uart2.buf);      //打印串口收到的信息
	strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//返回OK
	Clear_Buffer();	
	while(strx==NULL)
	{
			//Gui_DrawFont_GBK16(16,70,RED,WHITE, "全程技术支持");
			Gui_DrawFont_GBK16(16,10,RED,WHITE, "CONNECT 4G..");
			printf("单片机正在连接模块......\r\n");
			Clear_Buffer();	
			Uart2_SendStr("ATE1\r\n"); 
			delay_ms(300);
			strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//返回OK
	}
	Gui_DrawFont_GBK16(16,10,RED,WHITE, "CONNECT [OK]");
	printf("****单片机和模块连接成功*****\r\n");
	Uart2_SendStr("ATI\r\n");//获取模块的版本
	delay_ms(300);
	Clear_Buffer();	
	
	Uart2_SendStr("AT+QCCID\r\n");//获取卡的序列号 
	delay_ms(300);
	Clear_Buffer();	
	
	Uart2_SendStr("AT+CGSN\r\n");//获取模块的imei
	delay_ms(300);
	Clear_Buffer();	
	
	Uart2_SendStr("AT+CIMI\r\n");//获取卡号，类似是否存在卡的意思，比较重要。
	delay_ms(300);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"460");//返460，表明识别到卡了
	while(strx==NULL)
	{
			Clear_Buffer();	
			Uart2_SendStr("AT+CIMI\r\n");//获取卡号，类似是否存在卡的意思，比较重要。
			delay_ms(300);
			strx=strstr((const char*)buf_uart2.buf,(const char*)"460");//返回OK,说明卡是存在的
	} 
	printf("我的卡号是 : %s \r\n",buf_uart2.buf+8);
	Clear_Buffer();	
	
	Gui_DrawFont_GBK16(16,10,RED,WHITE, "SIMCARD [OK]");
	
	Uart2_SendStr("AT+CGATT?\r\n");//查询激活状态
	delay_ms(300);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"+CGATT: 1");//返1
	Clear_Buffer();	
	while(strx==NULL)
	{
			Clear_Buffer();	
			Uart2_SendStr("AT+CGATT?\r\n");//获取激活状态
			delay_ms(300);
			strx=strstr((const char*)buf_uart2.buf,(const char*)"+CGATT: 1");//返回1,表明注网成功
	}
	
	Gui_DrawFont_GBK16(16,10,RED,WHITE, "REGISTER[OK]");
	
	
	Clear_Buffer();	
	Uart2_SendStr("AT+CSQ\r\n");//查看获取CSQ值
	delay_ms(300);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"+CSQ:");//返回CSQ
	if(strx)
	{
			printf("信号质量是:%s 注意：信号最大值是31 \r\n",buf_uart2.buf+14);      
	}
		
}


char* Get_4GIMEI_NUM(void)
{
		Clear_Buffer();	
		memset(IMEINUMBER,0,64);
		Uart2_SendStr("AT+CGSN=1\r\n");//查询激活状态
		delay_ms(100);
		strx=strstr((const char*)buf_uart2.buf,(const char*)"+CGSN: \"");//如果反馈错误就表示没有定位好
		if(strx)	//没有反馈错误就表示有经纬度了 然后来进行显示 反馈得到LOC就表示有位置了
		{
				strncpy(IMEINUMBER,strx+8,15); //获取维度数据
				Gui_DrawFont_GBK16(0,70,PINNK,WHITE, (u8*) IMEINUMBER);	//显示经度到液晶屏
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
		Uart2_SendStr("AT+QGPSLOC=0\r\n");//查询激活状态
		delay_ms(300);
		strx=strstr((const char*)buf_uart2.buf,(const char*)"+QGPSLOC:");//如果反馈错误就表示没有定位好
		
		if(strx)	//没有反馈错误就表示有经纬度了 然后来进行显示 反馈得到LOC就表示有位置了
		{
				//获取到经纬度
				strncpy(GPRMCSTR,strx+21,22);
				printf("模块定位好了经纬度是 %s\r\n",GPRMCSTR); //经纬度字符串直接打印到串口
				//+QGPSLOC: 035658.000,2603.8722N,11912.4159E,1.8,269.9,3,000.00,0.4,0.2,140821,10
				//维度26.06451
				memset(GPRMCSTRLAT,0,64);
				strncpy(GPRMCSTRLAT,strx+21,2); //获取维度数据
				strcat(GPRMCSTRLAT,".");
				strncpy(GPRMCSTRLAT+strlen((char*)GPRMCSTRLAT),strx+23,2); //获取维度数据
				strncpy(GPRMCSTRLAT+strlen((char*)GPRMCSTRLAT),strx+26,4); //获取维度数据
			
				//Gui_DrawFont_GBK16(0,90,RED,WHITE, (u8*) GPRMCSTRLAT);	//显示经度到液晶屏
				//经度 119.20694
				memset(GPRMCSTRLON,0,64);
				strncpy(GPRMCSTRLON,strx+32,3);	//获取经度
				strcat(GPRMCSTRLON,".");
				strncpy(GPRMCSTRLON+strlen((char*)GPRMCSTRLON),strx+35,2);	//获取经度
				strncpy(GPRMCSTRLON+strlen((char*)GPRMCSTRLON),strx+38,4);	//获取经度
				//Gui_DrawFont_GBK16(0,110,RED,WHITE, (u8*) GPRMCSTRLON);	//显示经度到液晶屏
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
		Uart2_SendStr("AT+QGPSGNMEA=\"RMC\"\r\n");//查询激活状态
		delay_ms(100);
		strx=strstr((const char*)buf_uart2.buf,(const char*)"$GNRMC");//返1
		errcount=0;	
		while(strx==NULL)
		{
				Clear_Buffer();	
				Uart2_SendStr("AT+QGPSGNMEA=\"RMC\"\r\n");//获取激活状态
				delay_ms(30);
				strx=strstr((const char*)buf_uart2.buf,(const char*)"$GNRMC");//返回1,表明注网成功
				errcount++;
				if(	errcount > 200)
				{
					errcount=0;
					GPIO_SetBits(GPIOA,GPIO_Pin_12); 	
					delay_ms(1000);
					GPIO_ResetBits(GPIOA,GPIO_Pin_12); 	 
					delay_ms(300);
					NVIC_SystemReset();	//一直没有获取到GPS定位就复位下
				}
		}
		sprintf(GPRMCSTR,"%s",strx);
		//Gui_DrawFont_GBK16(16,10,RED,WHITE, "STARTGPS[OK]");
		//Gui_DrawFont_GBK16(16,10,GREEN,WHITE, "GET  GPS INFO");
		Gui_DrawFont_GBK16(16,10,GREEN,WHITE, "GPS INFO [..]");
		Clear_Buffer();																							    //打印收到的GPS信息
		GPRMCSTR[2]=	'P';
		
		printf("============GETGPRMC==============\r\n%s",GPRMCSTR);		//打印GPRMC
		if(GPRMCSTR[17]=='A')
		{
			memset(latStrAF,0,64);
			memset(lonStrAF,0,64);
			Get_GPSdata();
			Gui_DrawFont_GBK16(0,90,BLUE,WHITE, (u8*) latStrAF);	//显示经度到液晶屏
			Gui_DrawFont_GBK16(0,110,BLUE,WHITE, (u8*) lonStrAF);	//显示经度到液晶屏
			Gui_DrawFont_GBK16(16,10,BLUE,WHITE, "GPS INFO [OK]");
			if(type==1)
				return latStrAF;
			if(type==2)
				return lonStrAF;
		}
		return 0;
}



/*****************************************************
下面是矫正火星坐标的
*****************************************************/


//解GPS析函数
//$GPRMC,134952.00,A,2603.9576,N,11912.4098,E,0.154,,280821,,,A,V*18
int Get_GPSdata()
{
		int i=0;
    strx=strstr((const char*)GPRMCSTR,(const char*)"A,");//获取纬度的位置
       if(strx)
        {
            for(i=0;i<9;i++)
            {
             latdata.Latitude[i]=strx[i+2];//获取纬度值2603.9576
            }
						strx=strstr((const char*)GPRMCSTR,(const char*)"N,");//获取经度值
						if(strx)
						{
								 for(i=0;i<10;i++)	//获取经度 11912.4098
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
			Getdata_Change(latdata.getstautus);//数据换算
			Clear_Buffer();
		 return 0;

}





/*************解析出经纬度数据,然后直接提交数据*******************/	

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
		     (float)(latdata.longitudedd[4]-0x30)/100+(float)(latdata.longitudedd[5]-0x30)/1000+(float)(latdata.longitudedd[6]-0x30)/10000)/60.0;//获取完整的数据
       
///////////////////////////////////////////
				for(i=0;i<2;i++)
						latdata.Latitudess[i]=latdata.Latitude[i];
				for(i=2;i<9;i++)
						latdata.Latitudedd[i-2]=latdata.Latitude[i];	
				 
			latdata.gpsdata[1]=(float)(latdata.Latitudess[0]-0x30)*10+(latdata.Latitudess[1]-0x30)\
		     +((latdata.Latitudedd[0]-0x30)*10+(latdata.Latitudedd[1]-0x30)+(float)(latdata.Latitudedd[3]-0x30)/10+\
		     (float)(latdata.Latitudedd[4]-0x30)/100+(float)(latdata.Latitudedd[5]-0x30)/1000+(float)(latdata.Latitudedd[6]-0x30)/10000)/60.0;//获取完整的数据b

	
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
纠偏结束
*****************************************************/

void Start_GPS(void)
{
		Clear_Buffer();	
		Uart2_SendStr("AT+QGPS=1\r\n");//查询激活状态
		delay_ms(300);
		strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//返1
		
		while(strx==NULL)
		{
				delay_ms(300);
				strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//返回1,表明注网成功
		}
		
		Gui_DrawFont_GBK16(16,10,RED,WHITE, "STARTGPS[OK]");
		Clear_Buffer();	
}

//关闭与OneNet Studio平台的连接
void Close_MQTT_aliyunStudio(void)
{
	//关闭之前建立的链接
	Uart2_SendStr("AT+QMTDISC=0\r\n");
	delay_ms(100);
	Clear_Buffer();
} 

//开启与Onenet Studio的连接
void Connect_MQTT_aliyunStudio(void)
{
	printf("\r\n开启与Onenet Studio的连接\r\n");
	Uart2_SendStr("AT+QMTCFG=\"recv/mode\",0,0,1\r\n");//配置接收模式
	delay_ms(300);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//返OK
	Clear_Buffer();	
	while(strx==NULL)
	{
			Clear_Buffer();	
			Uart2_SendStr("AT+QMTCFG=\"recv/mode\",0,0,1\r\n");//配置接收模式
			delay_ms(300);
			strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//返回OK,开启成功
	}
	Clear_Buffer();
	
	Uart2_SendStr("AT+QMTCFG=\"version\",0,4\r\n");//配置 MQTT 协议版本
	delay_ms(300);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//返OK
	Clear_Buffer();	
	while(strx==NULL)
	{
			Clear_Buffer();	
			Uart2_SendStr("AT+QMTCFG=\"version\",0,4\r\n");//配置 MQTT 协议版本
			delay_ms(300);
			strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//返回OK,开启成功
	}
	Clear_Buffer();
	
	memset(ATSTR,0,BUFLEN);
	sprintf(ATSTR,"AT+QMTOPEN=0,\"%s\",%s\r\n",ServerIP,Port); 
	Uart2_SendStr(ATSTR);//打开MQTT客户端网络
	delay_ms(300);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//返OK
	errcount=0;	
	while(strx==NULL)
	{
		errcount++;
		strx=strstr((const char*)buf_uart2.buf,(const char*)"OK");//返回OK
		delay_ms(100);
		if(errcount>100)     //超时退出死循环 表示服务器连接失败
		{
			errcount = 0;
			break;
		}
	}
	Clear_Buffer();
	
	Gui_DrawFont_GBK16(16,10,RED,WHITE, "  MQTT  [OK]");
	
	memset(ATSTR,0,BUFLEN);
	sprintf(ATSTR,"AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"\r\n",ClientID,Username,Password); 
	
	Uart2_SendStr(ATSTR);//连接客户端到MQTT服务器
	delay_ms(300);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"+QMTCONN: 0,0,0");//+QMTCONN: 0,0,0
	errcount=0;	
	while(strx==NULL)
	{
		errcount++;
		strx=strstr((const char*)buf_uart2.buf,(const char*)"+QMTCONN: 0,0,0");//返回+QMTCONN: 0,0,0
		Gui_DrawFont_GBK16(0,50,RED,WHITE, (u8*)strx);
		delay_ms(100);
		if(errcount>100)     //超时退出死循环 表示服务器连接失败
		{
			Gui_DrawFont_GBK16(16,10,RED,WHITE, "  MQTT  [NO]");
			errcount = 0;
			break;
		}
	}
	Clear_Buffer();
	printf("\r\n连接成功\r\n");
	Gui_DrawFont_GBK16(16,10,RED,WHITE, "ALICONNECTOK");
	
	memset(ATSTR,0,BUFLEN);
	sprintf(ATSTR,"AT+QMTSUB=0,1,\"%s\",2\r\n",Topic); 
	Uart2_SendStr(ATSTR);//订阅
	delay_ms(300);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"+QMTSUB: 0,1,0,1");//+QMTCONN: 0,0,0
	errcount=0;	
	while(strx==NULL)
	{
		errcount++;
		strx=strstr((const char*)buf_uart2.buf,(const char*)"+QMTSUB: 0,1,0,1");//返回+QMTCONN: 0,0,0
		delay_ms(30);
		if(errcount>100)     //超时退出死循环 表示订阅失败
		{
			Gui_DrawFont_GBK16(10,50,RED,WHITE, "subscribe NO");
			errcount = 0;
			break;
		}
	}
	Gui_DrawFont_GBK16(10,50,RED,WHITE, "subscribe OK");
	Clear_Buffer();
	printf("\r\n订阅成功\r\n");
}

//发送数据到OneNet Studio
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
	Uart2_SendStr(ATSTR);//发布消息
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
	printf("\r\n发送的JSON数据:%s\r\n",Buff);
	Uart2_SendStr(Buff);//发送数据
	delay_ms(100);
	strx=strstr((const char*)buf_uart2.buf,(const char*)"+QMTPUBEX: 0,0,0");
	if(strx!=NULL)
	{
		printf("\r\n数据发送成功\r\n");
	}
	Gui_DrawFont_GBK16(16,10,BLUE,WHITE, "Send DATA[OK]");
}


void Control_Led(void)
{

		strx=strstr((const char*)buf_uart2.buf,(const char*)"\"btn1\":1");//返回+QMTRECV:，表明接收到服务器发回的数据 开灯1
    if(strx)
		{
			Gui_DrawFont_GBK16(16,10,RED,WHITE, "REBOOTING STM32");  
//			LED1=0;
			NVIC_SystemReset();
			buf_uart2.index=0;
			memset(buf_uart2.buf,0,BUFLEN);
			return ;
		}
		strx=strstr((const char*)buf_uart2.buf,(const char*)"\"btn1\":0");//返回+QMTRECV:，表明接收到服务器发回的数据 关闭灯1
    if(strx)
		{
			Gui_DrawFont_GBK16(16,10,RED,WHITE, "TURNOFF LED1");  
//			LED1=1;
			buf_uart2.index=0;
			memset(buf_uart2.buf,0,BUFLEN);
			return ;
		}
		strx=strstr((const char*)buf_uart2.buf,(const char*)"\"btn2\":1");//返回+QMTRECV:，表明接收到服务器发回的数据 开灯2 
    if(strx)
		{
			Gui_DrawFont_GBK16(16,10,RED,WHITE, "TURN ON LED2");  
			LED2=0;
			buf_uart2.index=0;
			memset(buf_uart2.buf,0,BUFLEN);
			return ;
		}
		strx=strstr((const char*)buf_uart2.buf,(const char*)"\"btn2\":0");//返回+QMTRECV:，表明接收到服务器发回的数据 关闭灯2 
    if(strx)
		{
			Gui_DrawFont_GBK16(16,10,RED,WHITE, "TURNOFF LED2");  
			LED2=1;
			buf_uart2.index=0;
      memset(buf_uart2.buf,0,BUFLEN);
			return ;
		}
		strx=strstr((const char*)buf_uart2.buf,(const char*)"\"btn3\":1");//返回+QMTRECV:，表明接收到服务器发回的数据 开灯3 
    if(strx)
		{
			Gui_DrawFont_GBK16(16,10,RED,WHITE, "TURN ON LED3");  
			LED3=0;
			buf_uart2.index=0;
      memset(buf_uart2.buf,0,BUFLEN);
			return ;
		}
		strx=strstr((const char*)buf_uart2.buf,(const char*)"\"btn3\":0");//返回+QMTRECV:，表明接收到服务器发回的数据 关闭灯3
    if(strx)
		{
			Gui_DrawFont_GBK16(16,10,RED,WHITE, "TURNOFF LED3");  
			LED3=1;
			buf_uart2.index=0;
      memset(buf_uart2.buf,0,BUFLEN);
			return ;
		}
}









