#ifndef __image_H
#define __image_H	 
#include "sys.h"
//����OV7725��������װ��ʽԭ��,OV7725_WINDOW_WIDTH�൱��LCD�ĸ߶ȣ�OV7725_WINDOW_HEIGHT�൱��LCD�Ŀ��
//ע�⣺�˺궨��ֻ��OV7725��Ч
#define  OV7725_WINDOW_WIDTH		200 // <=320
#define  OV7725_WINDOW_HEIGHT		200 // <=240

enum
{
	horizontal=0,  //ˮƽ����
	vertical		//��ֱ����
	
};
enum
{
	forward=0,
	turn_left,
	turn_right,
	stop,
	back
};

void camera_refresh(void);
int flight_data(void);
int Tracking_mode(uint16_t state_buff[2][100]);
#endif


