#ifndef __BC26_H
#define __BC26_H	
#include "usart.h"
#include <stm32f10x.h>
#include "delay.h"

void Clear_Buffer(void);//清空缓存	
void CSTX_4G_Init(void);

void Close_MQTT_aliyunStudio(void);
void Connect_MQTT_aliyunStudio(void);
void SendData_aliyunStudio(char *ID,char *Data);


void Control_Led(void);

void Start_GPS(void);
char *Get_GPS_RMC(char type);
char* Get_GPS_LOC(char type);
char* Get_4GIMEI_NUM(void); //获取IMEI号

typedef struct
{
    uint8_t CSQ;    
    uint8_t Socketnum;   //编号
    uint8_t reclen;   //获取到数据的长度
    uint8_t res;      
    uint8_t recdatalen[10];
    uint8_t recdata[100];
    uint8_t netstatus;//网络指示灯
} CSTX_4G;

#endif







