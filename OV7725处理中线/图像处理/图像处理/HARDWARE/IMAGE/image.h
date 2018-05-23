#ifndef __image_H
#define __image_H	 
#include "sys.h"
//由于OV7725传感器安装方式原因,OV7725_WINDOW_WIDTH相当于LCD的高度，OV7725_WINDOW_HEIGHT相当于LCD的宽度
//注意：此宏定义只对OV7725有效
#define  OV7725_WINDOW_WIDTH		200 // <=320
#define  OV7725_WINDOW_HEIGHT		200 // <=240

enum
{
	horizontal=0,  //水平方向
	vertical		//竖直方向
	
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


