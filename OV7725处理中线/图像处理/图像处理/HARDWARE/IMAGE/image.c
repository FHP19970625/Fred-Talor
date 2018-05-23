#include "lcd.h" 
#include "string.h"
#include "ov7725.h"
#include "timer.h"
#include "exti.h"
#include "usmart.h"
#include "image.h"
#include "LCD.h"
#include "LED.h"
#include "delay.h"
extern long int Camera_Data_Buff[2][100];
extern  uint16_t camera_dis_buf[100][100];
extern  uint16_t camera_dis2_buf[100][100];
extern  uint16_t camera_dis3_buf[100][100];
extern uint16_t camera_min_data[2][100];
void camera_refresh(void)
{
	u16 color;	
	uint16_t i, j ,buff=0; 
	uint16_t Camera_Data,Camera_Data_old;
	u16 fiter_buff[4]={0};
		LCD_Scan_Dir(U2D_R2L);		//从上到下,从左到右 
	//LCD_Set_Window((lcddev.width-OV7725_WINDOW_WIDTH),(lcddev.height-OV7725_WINDOW_HEIGHT),OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT);//将显示区域设置到屏幕中央
	//	LCD_WriteRAM_Prepare();     //开始写入GRAM	
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
///////////////////////////////////////////////////////////////			
				if(i>=100&&i<200&&j>=100&&j<200)
			{
				camera_dis_buf[j-100][i-100]= Camera_Data;
			}			
			Camera_Data_old= Camera_Data;
//////////////////////////////////////////////////////////////			
			// LCD_WriteRAM(Camera_Data);				
		}   
	}						
		  EXTI_ClearITPendingBit(EXTI_Line8);  //清除LINE8上的中断标志位
}


int midline_data(void)
{
	u8 i,j;
	u16 old_data,old_data2;
	u8 left_data,left_data_old,right_data;
	u8 left_flag=0;
	
	u8 vertical_data,vertical_data_old,horizontal_data,horizontal_data_old;
	u8 vertical_flag=0,horizontal_flag=0;
	
	for(i=0;i<100;i++,left_flag=0,vertical_flag=0,horizontal_flag=0)      
	{
		for(j=0;j<100;j++)
		{
			if((camera_dis_buf[i][j] != old_data) && j>=2)							//水平方向的中线数据
			{
				camera_dis2_buf[i][j] = WHITE;
				horizontal_data=j;
				horizontal_flag++;
				if((horizontal_flag == 2)&& (horizontal_data - horizontal_data_old < 30))
				{
					horizontal_flag=0;
						camera_dis3_buf[i][((horizontal_data + horizontal_data_old) / 2)] = BLACK;
					  camera_min_data[horizontal][i] = (horizontal_data + horizontal_data_old) / 2 ;
						//LCD_Fast_DrawPoint(camera_min_data[horizontal][i]+200,i+200,BLACK);
					 //  cammera_hen_date[horizontal][i]=(horizontal_data + horizontal_data_old) / 2;
				}
				else 
				{
					horizontal_flag=1;
					camera_dis2_buf[i][j] = BLACK;
				}
				horizontal_data_old= horizontal_data ;
			}
			else	
			{
				
				camera_dis3_buf[i][j] = WHITE;
				camera_dis2_buf[i][j] = BLACK;
			}

			old_data = camera_dis_buf[i][j];
			
		}
	}
	for(i=0;i<100;i++,left_flag=0,vertical_flag=0,horizontal_flag=0)      
	{
		for(j=0;j<100;j++)
		{
			if((  camera_dis_buf[j][i] != old_data2) && j>=2)                   //竖直方向的中线数据
			{
				camera_dis2_buf[j][i] = WHITE;
				vertical_data = j ;
				vertical_flag++;
				
				if((vertical_flag == 2)&& (vertical_data - vertical_data_old < 30))
				{
					vertical_flag=0;

						camera_dis3_buf[((vertical_data + vertical_data_old) / 2)][i] = BLACK ;
						camera_min_data[vertical][j] = (vertical_data + vertical_data_old) / 2 ;
					//	LCD_Fast_DrawPoint(i+200,camera_min_data[vertical][i]+200,BLACK);
				//	cammera_hen_date[vertical][i]=(vertical_data + vertical_data_old) / 2;
	
				}
				else
				{
					vertical_flag=1;
//				camera_dis2_buf[j][i] = BLACK;
//					camera_dis3_buf[j][i] = WHITE;
					
				}
				vertical_data_old = vertical_data ;
			
				
			}
//			else
//			{
//			
//				camera_dis3_buf[j][i] = WHITE;
//				camera_dis2_buf[i][j] = BLACK;
//			}
				old_data2 = camera_dis_buf[j][i];
		}
	}

	return 0 ;
}
	


int flight_data(void)
{
	u16 i,j;

	
//	for(i=0;i<200;i++)  // 清楚中线的显示，刷屏
//	{
//		for(j=0;j<200;j++)
//		{
//			LCD_Fast_DrawPoint(i,j,WHITE);
//		}
//	}
	for(j=0;j<100;j++)     // 显示原始图像
	{
		for(i=0;i<100;i++)
		{
			LCD_Fast_DrawPoint(j+100,i+100,camera_dis_buf[i][j]);
		}
	}
	midline_data();
	for(j=0;j<100;j++)   
	{
		for(i=0;i<100;i++)
		{
			LCD_Fast_DrawPoint(j+200,i+200,camera_dis3_buf[i][j]);
		}
		
	}	
	
  return 0;
}



int Tracking_mode(uint16_t state_buff[2][100])
{
	int i;
	static int turn_flag=0;

	for(i=10;i<90;i++)
	{
		if(state_buff[vertical][i] == 0)        //前方都有白点
		{
			if((state_buff[vertical][i+1] == 0)&&(state_buff[vertical][i-1] == 0))		//有连续的白点
			{
				if(state_buff[vertical][80] == 0)               //确定前方没有黑线
				{
					if(state_buff[horizontal][10]&&state_buff[horizontal][90])
					{
						LCD_ShowString(400,60,200,16,16,"stop");
						return stop;
					}
					else if(state_buff[horizontal][10]&&turn_flag == 0)
					{
						LCD_ShowString(300,60,200,16,16,"turn right");	
						turn_flag++;
						return turn_left;
					}
					else if(state_buff[horizontal][90]&&turn_flag== 0)
					{
						LCD_ShowString(500,60,200,16,16,"turn left");	
						turn_flag++;
						return turn_right;
					}
				}
			}
		}
		else
		{
			turn_flag=0;
			LCD_ShowString(500,60,200,16,16,"Move");
			return forward;//十字形
		}
	}
  // return forward;//十字形
}

//int Tracking_mode(uint16_t state_buff[2][100])
//{
//	int i;
//	static int turn_flag=0;
//	//LCD_ShowString(300,60,200,16,16,"OV7725 Init OK");
//	for(i=30;i<50;i++)
//	{
//		if(state_buff[vertical][i] == 0)        //前方都有白点
//		{
//			if((state_buff[vertical][i+5] == 0)&&(state_buff[vertical][i-5] == 0))		//有连续的白点
//			{
//				if(state_buff[vertical][20] == 0)               //确定前方没有黑线
//				{
//					if(state_buff[horizontal][10]&&state_buff[horizontal][90])
//					{
//						LCD_ShowString(400,60,200,16,16,"stop");			
//					}
//		       if(state_buff[horizontal][10]&&state_buff[horizontal][90]==0)
//					{
//						LCD_ShowString(300,60,200,16,16,"turn right");											
//					}
//				  if(state_buff[horizontal][90]&&state_buff[horizontal][10]==0)
//					{
//						LCD_ShowString(500,60,200,16,16,"turn left");	
//					}
//				}
//			}
//		}
//	}
//	return forward;
//}

