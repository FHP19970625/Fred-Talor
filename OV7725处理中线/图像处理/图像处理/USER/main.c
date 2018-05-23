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
 ALIENTEK战舰STM32开发板实验35
 摄像头OV7725和OV7670 实验
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/
//更新LCD显示(OV7725)

long int Camera_Data_Buff[2][100]={0};
uint16_t camera_dis_buf[100][100];
uint16_t camera_dis2_buf[100][100];
uint16_t camera_dis3_buf[100][100];
uint16_t camera_min_data[2][100];
extern u8 ov_sta;	//在exit.c里 面定义




 int main(void)
{	
	delay_init();	    	 	//延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(9600);	 		//串口初始化为 115200
	LCD_Init();			   		//初始化LCD  
  OV7725_Init();//初始化OV7725 
	LCD_Clear (WHITE);
	POINT_COLOR=RED;			//设置字体为红色 
	LCD_ShowString(60,300,200,16,16,"OV7725 Init...");	
 	LCD_ShowString(60,300,200,16,16,"OV7725 Init OK");	  
  while(1)	
	{  
		while (1)
		{
		 	OV7725_Window_Set(OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT,0);//VGA模式输出
					break;
		}
		 OV7725_CS=0;
		  break;
	}
	LCD_Clear (WHITE);
	TIM3_Int_Init(19,7199);	//10Khz计数频率,1秒钟中断									  
	EXTI8_Init();				//使能定时器捕获	
 	while(1)
	{	
		if(ov_sta==1)
	   {
			camera_refresh();//更新显示
			// LCD_DrawLine(0,2,100,camera_dis_buf[0]); 
			flight_data(); 
			
			ov_sta=0;					//开始下一次采集
	  } 
		 Tracking_mode(camera_min_data);
	}	  
}













