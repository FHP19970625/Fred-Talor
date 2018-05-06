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
 ALIENTEKս��STM32������ʵ��35
 ����ͷOV7725��OV7670 ʵ��
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/


#define  OV7725 1

//����OV7725��������װ��ʽԭ��,OV7725_WINDOW_WIDTH�൱��LCD�ĸ߶ȣ�OV7725_WINDOW_HEIGHT�൱��LCD�Ŀ��
//ע�⣺�˺궨��ֻ��OV7725��Ч
#define  OV7725_WINDOW_WIDTH		480 // <=320
#define  OV7725_WINDOW_HEIGHT		320 // <=240

extern u8 ov_sta;	//��exit.c�� �涨��
extern u8 ov_frame;	//��timer.c���涨�� 

//����LCD��ʾ(OV7725)
void camera_refresh(void)
{

	u16 x,y;//xΪ����yΪ��320*240 Ϊ��x��
 	u16 color;	
	
	if(ov_sta)
	{
		LCD_Scan_Dir(U2D_R2L);		//���ϵ���,������ 
		LCD_SetCursor(0X00,0x0000);	//���ù��λ�� 
		LCD_WriteRAM_Prepare();     //��ʼд��GRAM	
		FIFO_PREPARE; 			//FIFO׼��
		 for(y=0;y<320;y++)
		   {
		    for(x=0;x<480;x++)
						{			
							OV7725_RCK_L;
							color=GPIOC->IDR&0XFF;	//YUYV���������  �����ڶ����ֽ�
							OV7725_RCK_H; 
							OV7725_RCK_L;
							OV7725_RCK_H;
            if((x>0&&x<=480)&&(y>0&&y<=320))//480*320		
							{								
							   if(color>0x60) color=0xff;//��ɫ             
										else
									{					   color=0x00;//��ɫ							

								}
     					LCD->LCD_RAM=GRAY_2_RGB565(color); //�Ҷ�ת������
							}
							else LCD->LCD_RAM=GRAY_2_RGB565(0xff);
							   
						}   
				}						
		  EXTI_ClearITPendingBit(EXTI_Line8);  //���LINE8�ϵ��жϱ�־λ
		  ov_sta=0;					//��ʼ��һ�βɼ�
					ov_frame++; 
		  LCD_Scan_Dir(DFT_SCAN_DIR);	//�ָ�Ĭ��ɨ�跽��  
 }
}
 int main(void)
{	
	delay_init();	    	 	//��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(9600);	 		//���ڳ�ʼ��Ϊ 115200
	LCD_Init();			   		//��ʼ��LCD  
  OV7725_Init();//��ʼ��OV7725 
	POINT_COLOR=RED;			//��������Ϊ��ɫ 
	LCD_ShowString(60,230,200,16,16,"OV7725 Init...");	
 	LCD_ShowString(60,230,200,16,16,"OV7725 Init OK");	  
  while(1)	
	{  
		while (1)
		{
		 	OV7725_Window_Set(OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT);//VGAģʽ���
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
			camera_refresh();//������ʾ
	}	  
}













