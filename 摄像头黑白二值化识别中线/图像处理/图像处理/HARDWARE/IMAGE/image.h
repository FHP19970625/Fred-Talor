#ifndef __image_H
#define __image_H	 
#
//由于OV7725传感器安装方式原因,OV7725_WINDOW_WIDTH相当于LCD的高度，OV7725_WINDOW_HEIGHT相当于LCD的宽度
//注意：此宏定义只对OV7725有效
#define  OV7725_WINDOW_WIDTH		192 // <=320
#define  OV7725_WINDOW_HEIGHT		128 // <=240

enum
{
	horizontal=0,  //水平方向
	vertical		//竖直方向
	
};


void camera_refresh(void);

#endif


