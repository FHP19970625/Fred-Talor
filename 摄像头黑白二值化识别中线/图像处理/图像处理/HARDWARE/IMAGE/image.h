#ifndef __image_H
#define __image_H	 
#
//����OV7725��������װ��ʽԭ��,OV7725_WINDOW_WIDTH�൱��LCD�ĸ߶ȣ�OV7725_WINDOW_HEIGHT�൱��LCD�Ŀ��
//ע�⣺�˺궨��ֻ��OV7725��Ч
#define  OV7725_WINDOW_WIDTH		192 // <=320
#define  OV7725_WINDOW_HEIGHT		128 // <=240

enum
{
	horizontal=0,  //ˮƽ����
	vertical		//��ֱ����
	
};


void camera_refresh(void);

#endif


