#ifndef _key_H
#define _key_H

#include "sys.h"

//ʹ��λ��������
#define K1 PAin(0)
#define K2 PAin(8)

//�����������ֵ  
#define Key1 1
#define Key2 2
 
void KEY_Init(void);
u8 KEY_Scan(u8 mode);

#endif

