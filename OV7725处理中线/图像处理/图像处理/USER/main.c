#include "led.h"
#include "delay.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 
#include "string.h"
#include "ov7725.h"
#include "timer.h"
#include "exti.h"
#include "image.h"

/************************************************
 ALIENTEKս��STM32������ʵ��35
 ����ͷOV7725��OV7670 ʵ��
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/
//����LCD��ʾ(OV7725)

long int Camera_Data_Buff[2][100]={0};
uint16_t camera_dis_buf[100][100];
uint16_t camera_dis2_buf[100][100];
uint16_t camera_dis3_buf[100][100];
uint16_t camera_min_data[2][100];
extern u8 ov_sta;	//��exit.c�� �涨��




 int main(void)
{	
	delay_init();	    	 	//��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(9600);	 		//���ڳ�ʼ��Ϊ 115200
	LCD_Init();			   		//��ʼ��LCD  
  OV7725_Init();//��ʼ��OV7725 
	LCD_Clear (WHITE);
	POINT_COLOR=RED;			//��������Ϊ��ɫ 
	LCD_ShowString(60,300,200,16,16,"OV7725 Init...");	
 	LCD_ShowString(60,300,200,16,16,"OV7725 Init OK");	  
  while(1)	
	{  
		while (1)
		{
		 	OV7725_Window_Set(OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT,0);//VGAģʽ���
					break;
		}
		 OV7725_CS=0;
		  break;
	}
	LCD_Clear (WHITE);
	TIM3_Int_Init(19,7199);	//10Khz����Ƶ��,1�����ж�									  
	EXTI8_Init();				//ʹ�ܶ�ʱ������	
 	while(1)
	{	
		if(ov_sta==1)
	   {
			camera_refresh();//������ʾ
			// LCD_DrawLine(0,2,100,camera_dis_buf[0]); 
			flight_data(); 
			
			ov_sta=0;					//��ʼ��һ�βɼ�
	  } 
		 Tracking_mode(camera_min_data);
	}	  
}













