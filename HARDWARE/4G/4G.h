#ifndef __BC26_H
#define __BC26_H	
#include "usart.h"
#include <stm32f10x.h>
#include "delay.h"

void Clear_Buffer(void);//��ջ���	
void CSTX_4G_Init(void);

void Close_MQTT_aliyunStudio(void);
void Connect_MQTT_aliyunStudio(void);
void SendData_aliyunStudio(char *ID,char *Data);


void Control_Led(void);

void Start_GPS(void);
char *Get_GPS_RMC(char type);
char* Get_GPS_LOC(char type);
char* Get_4GIMEI_NUM(void); //��ȡIMEI��

typedef struct
{
    uint8_t CSQ;    
    uint8_t Socketnum;   //���
    uint8_t reclen;   //��ȡ�����ݵĳ���
    uint8_t res;      
    uint8_t recdatalen[10];
    uint8_t recdata[100];
    uint8_t netstatus;//����ָʾ��
} CSTX_4G;

#endif







