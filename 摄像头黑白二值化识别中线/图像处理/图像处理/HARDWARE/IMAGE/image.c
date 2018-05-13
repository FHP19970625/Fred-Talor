#include "lcd.h" 
#include "string.h"
#include "ov7725.h"
#include "timer.h"
#include "exti.h"
#include "usmart.h"
#include "image.h"
extern long int Camera_Data_Buff[2][100];
extern  uint16_t camera_dis_buf[100][100];
void camera_refresh(void)
{
	u16 color;	
	uint16_t i, j ,buff=0; 
	uint16_t Camera_Data,Camera_Data_old;
	u16 fiter_buff[4]={0};
		LCD_Scan_Dir(U2D_R2L);		//���ϵ���,������ 
	//	LCD_Set_Window((lcddev.width-OV7725_WINDOW_WIDTH),(lcddev.height-OV7725_WINDOW_HEIGHT),OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT);//����ʾ�������õ���Ļ����
		LCD_WriteRAM_Prepare();     //��ʼд��GRAM	
		FIFO_PREPARE; 			//FIFO׼��
		 for(j=0;j<OV7725_WINDOW_HEIGHT;j++)
		   {
		    for(i=0;i<OV7725_WINDOW_WIDTH;i++)
						{			
						READ_FIFO_PIXEL(Camera_Data);		/* ��FIFO����һ��rgb565���ص�Camera_Data���� */
            	if(Camera_Data> 0X60)
				    Camera_Data=WHITE ;
			    else Camera_Data=BLACK;
							
/**** 1*3 �˲�****/							  
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
				if(i>=100&&i<196&&j>=100&&j<128)
			{
				camera_dis_buf[j-100][i-100]= Camera_Data;
			}			
			Camera_Data_old= Camera_Data;
//////////////////////////////////////////////////////////////			
			// LCD_WriteRAM(Camera_Data);				
		}   
	}						
		  EXTI_ClearITPendingBit(EXTI_Line8);  //���LINE8�ϵ��жϱ�־λ
}


