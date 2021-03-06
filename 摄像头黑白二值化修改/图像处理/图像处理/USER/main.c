#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 
#include "string.h"
#include "ov7725.h"
#include "timer.h"
#include "exti.h"
#include "usmart.h"


/************************************************
 ALIENTEK战舰STM32开发板实验35
 摄像头OV7725和OV7670 实验
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/


#define  OV7725 1

//由于OV7725传感器安装方式原因,OV7725_WINDOW_WIDTH相当于LCD的高度，OV7725_WINDOW_HEIGHT相当于LCD的宽度
//注意：此宏定义只对OV7725有效
#define  OV7725_WINDOW_WIDTH		192 // <=320
#define  OV7725_WINDOW_HEIGHT		128 // <=240

extern u8 ov_sta;	//在exit.c里 面定义
extern u8 ov_frame;	//在timer.c里面定义 
long int Camera_Data_Buff[2][100]={0};
uint16_t camera_dis_buf[100][100];
unsigned  char deal_buff[6][128];
//更新LCD显示(OV7725)
void camera_refresh(void)
{

	//u16 x,y;//x为长，y为宽，320*240 为长x宽
	u16 color;	
	uint16_t i, j ,buff=0; 
	uint16_t Camera_Data,Camera_Data_old;
	u16 fiter_buff[4]={0};
	if(ov_sta==1)
	{
		LCD_Scan_Dir(U2D_R2L);		//从上到下,从左到右 
		//LCD_SetCursor(0X00,0x0000);	//设置光标位置 
		LCD_Set_Window((lcddev.width-OV7725_WINDOW_WIDTH)/2,(lcddev.height-OV7725_WINDOW_HEIGHT)/2,OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT);//将显示区域设置到屏幕中央
		LCD_WriteRAM_Prepare();     //开始写入GRAM	
		FIFO_PREPARE; 			//FIFO准备
		 for(j=0;j<OV7725_WINDOW_HEIGHT;j++)
		   {
		    for(i=0;i<OV7725_WINDOW_WIDTH;i++)
						{			
						READ_FIFO_PIXEL(Camera_Data);		/* 从FIFO读出一个rgb565像素到Camera_Data变量 */
            	if(Camera_Data> 0X60)
				    Camera_Data=WHITE ;
			    else Camera_Data=BLACK;
							
/**** 1*3 滤波****/							  
       if(j<3)
				fiter_buff[j]= Camera_Data;
			else
			{
				for(buff=0;buff<3;buff++)
				{
					fiter_buff[0]= fiter_buff[1];
					fiter_buff[1]= fiter_buff[2];
					fiter_buff[2]= Camera_Data;
				}
				
				if(fiter_buff[0]>fiter_buff[1])
				{
					fiter_buff[3]= fiter_buff[0];
					fiter_buff[0]= fiter_buff[1];
					fiter_buff[1]= fiter_buff[3];
					
					if(fiter_buff[1]>fiter_buff[2])
					{
						
						fiter_buff[1]= fiter_buff[2];
						fiter_buff[2]= fiter_buff[3];
					}
		
				}
				Camera_Data=fiter_buff[1];
			}		
			 LCD_WriteRAM(Camera_Data);				
		}   
	}						
		  EXTI_ClearITPendingBit(EXTI_Line8);  //清除LINE8上的中断标志位
		  ov_sta=0;					//开始下一次采集
			ov_frame++; 
		 // LCD_Scan_Dir(DFT_SCAN_DIR);	//恢复默认扫描方向  
 }
}
 int main(void)
{	
	delay_init();	    	 	//延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(9600);	 		//串口初始化为 115200
	LCD_Init();			   		//初始化LCD  
  OV7725_Init();//初始化OV7725 
	POINT_COLOR=RED;			//设置字体为红色 
	LCD_ShowString(60,230,200,16,16,"OV7725 Init...");	
 	LCD_ShowString(60,230,200,16,16,"OV7725 Init OK");	  
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
	TIM3_Int_Init(1000,7199);	//10Khz计数频率,1秒钟中断									  
	EXTI8_Init();				//使能定时器捕获	
	LCD_Clear(BLACK);
 	while(1)
	{	
			camera_refresh();//更新显示
	}	  
}













