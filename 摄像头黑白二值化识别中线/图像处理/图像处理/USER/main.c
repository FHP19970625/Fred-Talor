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


//int midline_deal(void)
//{
//	u8 i,j;
//	u16 old_data,old_data2;
//	u8 left_data,left_data_old,right_data;
//	u8 left_flag=0;
//	
//	u8 vertical_data,vertical_data_old,horizontal_data,horizontal_data_old;
//	u8 vertical_flag=0,horizontal_flag=0;
//	
//	for(i=0;i<100;i++,left_flag=0,vertical_flag=0,horizontal_flag=0)      
//	{
//		for(j=0;j<100;j++)
//		{
//			if((camera_dis_buf[i][j] != old_data) && j>=2)							//ˮƽ�������������
//			{
//				camera_dis2_buf[i][j] = WHITE;
//				horizontal_data=j;
//				horizontal_flag++;
//				if((horizontal_flag == 2)&& (horizontal_data - horizontal_data_old < 30))
//				{
//					horizontal_flag=0;
//					
//						camera_min_data[horizontal][i] = (horizontal_data + horizontal_data_old) / 2 ;
//						camera_dis3_buf[i][((horizontal_data + horizontal_data_old) / 2)] = BLACK;
//						LCD_Fast_DrawPoint(camera_min_data[horizontal][i]+100,i+100,BLACK);
//					if(i>0)
//					{
//						if((camera_min_data[horizontal][i] > camera_min_data[horizontal][i-1] - 5 )
//								&& (camera_min_data[horizontal][i] < camera_min_data[horizontal][i-1] + 5))
//						{
//							camera_dis3_buf[i][((horizontal_data + horizontal_data_old) / 2)] = BLACK;
//						}
//						else 
//						{
//							camera_dis3_buf[i][(camera_min_data[horizontal][i-1])] = BLACK;
//							camera_min_data[horizontal][i] = camera_min_data[horizontal][i-1];
//						}
//						
//					}
//				}
//				else 
//				{
//					horizontal_flag=1;
//					camera_dis2_buf[i][j] = BLACK;
//				}
//				horizontal_data_old= horizontal_data ;
//			}
//			else	
//			{
//				
//				camera_dis3_buf[i][j] = WHITE;
//				camera_dis2_buf[i][j] = BLACK;
//			}
//			
//			if((  camera_dis_buf[j][i] != old_data2) && j>=2)                   //��ֱ�������������
//			{
//				camera_dis2_buf[j][i] = WHITE;
//				vertical_data = j ;
//				vertical_flag++;
//				
//				if((vertical_flag == 2)&& (vertical_data - vertical_data_old < 30))
//				{
//					vertical_flag=0;

//						camera_min_data[vertical][i] = (vertical_data + vertical_data_old) / 2 ;
//					  camera_dis3_buf[((vertical_data + vertical_data_old) / 2)][i] = BLACK ;
//						LCD_Fast_DrawPoint(i+100,camera_min_data[vertical][i]+100,BLACK);
//	        if(i>0)
//					{
//						if((camera_min_data[vertical][i] > camera_min_data[vertical][i-1] - 5 )
//								&& (camera_min_data[vertical][i] < camera_min_data[vertical][i-1] + 5))
//						{
//							camera_dis3_buf[i][((vertical_data + vertical_data_old) / 2)] = BLACK;
//						}
//						else 
//						{
//							camera_dis3_buf[i][(camera_min_data[horizontal][i-1])] = BLACK;
//							camera_min_data[vertical][i] = camera_min_data[vertical][i-1];
//						}
//						
//					}
//				}
//				else
//				{
//					vertical_flag=1;
//					camera_dis2_buf[i][j] = BLACK;
//				}
//				vertical_data_old = vertical_data ;
//			
//				
//			}
//			else
//				{
//				
//					camera_dis3_buf[i][j] = WHITE;
//					camera_dis2_buf[i][j] = BLACK;
//				}
//			old_data = camera_dis_buf[i][j];
//			old_data2 = camera_dis_buf[j][i];
//		}
//  }
//	return 0 ;
//}

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
			if((camera_dis_buf[i][j] != old_data) && j>=2)							//ˮƽ�������������
			{
				camera_dis2_buf[i][j] = WHITE;
				horizontal_data=j;
				horizontal_flag++;
				if((horizontal_flag == 2)&& (horizontal_data - horizontal_data_old < 30))
				{
					horizontal_flag=0;
						camera_min_data[horizontal][i] = (horizontal_data + horizontal_data_old) / 2 ;
						camera_dis3_buf[i][((horizontal_data + horizontal_data_old) / 2)] = BLACK;
						LCD_Fast_DrawPoint(camera_min_data[horizontal][i]+100,i+100,BLACK);
					
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
			if((  camera_dis_buf[j][i] != old_data2) && j>=2)                   //��ֱ�������������
			{
				camera_dis2_buf[j][i] = WHITE;
				vertical_data = j ;
				vertical_flag++;
				
				if((vertical_flag == 2)&& (vertical_data - vertical_data_old < 30))
				{
					vertical_flag=0;

						camera_dis3_buf[((vertical_data + vertical_data_old) / 2)][i] = BLACK ;
						camera_min_data[vertical][i] = (vertical_data + vertical_data_old) / 2 ;
						LCD_Fast_DrawPoint(i+100,camera_min_data[vertical][i]+100,BLACK);
	
				}
				else
				{
					vertical_flag=1;
//					camera_dis2_buf[j][i] = BLACK;
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

	
	for(i=100;i<200;i++)  // ������ߵ���ʾ��ˢ��
	{
		for(j=100;j<200;j++)
		{
			LCD_Fast_DrawPoint(i,j,WHITE);
		}
	}
	for(i=0;i<100;i++)     // ��ʾԭʼͼ��
	{
		for(j=0;j<100;j++)
		{
			LCD_Fast_DrawPoint(j,i,camera_dis_buf[i][j]);
		}
	}
	
//	midline_deal();        //������ȡ ��������ȡ
	midline_data();
	
//	for(i=0;i<100;i++)   
//	{

//		
//			
//		camera_dis3_buf[i][camera_min_data[horizontal][i]] = BLACK ;
//		camera_dis3_buf[camera_min_data[vertical][i]][i] = BLACK ;


//	}
	for(i=0;i<100;i++)   
	{
		for(j=0;j<100;j++)
		{
//			LCD_Fast_DrawPoint(j,i+100,camera_dis2_buf[i][j]);
			LCD_Fast_DrawPoint(j+100,i,camera_dis3_buf[i][j]);
		}
	}
}

 int main(void)
{	
	delay_init();	    	 	//��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(9600);	 		//���ڳ�ʼ��Ϊ 115200
	LCD_Init();			   		//��ʼ��LCD  
  OV7725_Init();//��ʼ��OV7725 
	LCD_Clear (WHITE);
	POINT_COLOR=BLACK;			//��������Ϊ��ɫ 
	LCD_ShowString(60,230,200,16,16,"OV7725 Init...");	
 	LCD_ShowString(60,230,200,16,16,"OV7725 Init OK");	  
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
	TIM3_Int_Init(1000,7199);	//10Khz����Ƶ��,1�����ж�									  
	EXTI8_Init();				//ʹ�ܶ�ʱ������	
	LCD_Clear(BLACK);
 	while(1)
	{	
		if(ov_sta==1)
	   {
			camera_refresh();//������ʾ
		  
			 
			 
	//		 LCD_DrawLine(0,2,100,camera_dis_buf[0]);
			 
			 
			 
			flight_data(); 
			ov_sta=0;					//��ʼ��һ�βɼ�
		
		
	  }
	}	  
}













