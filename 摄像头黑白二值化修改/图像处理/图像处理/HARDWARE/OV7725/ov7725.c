#include "sys.h"
#include "ov7725.h"
#include "ov7725config.h"	  
#include "delay.h"
#include "usart.h"			 
#include "sccb.h"	
//////////////////////////////////////////////////////////////////////////////////
//ALIENTEKս��STM32������
//OV7725 ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2017/11/1
//�汾��V1.0		    							    							  
//////////////////////////////////////////////////////////////////////////////////

OV7725_CONFIG ov7725_config;		    			    
////��ʼ��OV7725
////����0:�ɹ�
////��������ֵ:�������
//void config_ov7725_OutPut(u16 xsta,u16 ysta,u8 ouput_mode){
//	int i=0;
//	ov7725_config.xsta = xsta;
//	ov7725_config.ysta = ysta;
//	//ov7725_config.width = width;
//	//ov7725_config.height = height;
//	ov7725_config.mode = ouput_mode;
//	
//  
//	if(ouput_mode){		//�ڰ����
//		for(i=0;i<sizeof(ov7725_init_reg_YUV)/sizeof(ov7725_init_reg_YUV[0])/2;i++)
//		{
//			SCCB_WR_Reg(ov7725_init_reg_YUV[i][0],ov7725_init_reg_YUV[i][1]);
//			delay_ms(2);
//		}

//	}else{				//RGB565���
//		for(i=0;i<sizeof(ov7725_init_reg_tb1)/sizeof(ov7725_init_reg_tb1[0])/2;i++)
//		{
//			SCCB_WR_Reg(ov7725_init_reg_tb1[i][0],ov7725_init_reg_tb1[i][1]);
//			delay_ms(2);
//		}
//	}
//	//OV7725_Window_Set(176,10,width,height);	//���ô���	
//	//LCD_Clear(WHITE);	
//}
////��ʼ��OV7725
////����0:�ɹ�
////��������ֵ:�������
u8 OV7725_Init(void)
{
//	u16 i=0;
//	u16 reg=0;
	u8 temp;
	//����IO
 	GPIO_InitTypeDef  GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOG|RCC_APB2Periph_AFIO, ENABLE);//ʹ����ض˿�ʱ��
 
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8; 	//PA8 ���� ����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
		
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;	 // �˿�����
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_4);	

	
	GPIO_InitStructure.GPIO_Pin  = 0xff; //PC0~7 ���� ����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
	 
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD,GPIO_Pin_6);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_14|GPIO_Pin_15;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 	GPIO_Init(GPIOG, &GPIO_InitStructure);
	GPIO_SetBits(GPIOG,GPIO_Pin_14|GPIO_Pin_15);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	//SWD
 
	SCCB_Init();        		//��ʼ��SCCB ��IO��	
 	if(SCCB_WR_Reg(0x12,0x80))return 1;	//��λOV7725
	delay_ms(50); 
	 	temp=SCCB_RD_Reg(0x0b);   
//	if(temp!= OV7725_ID)return 2;  
// 	temp=SCCB_RD_Reg(0x0a);   
//	if(temp!=0x76)return 2;

	//��ʼ������
	if(temp == OV7725_ID)
	{
	  // config_ov7725_OutPut(320,480,1);
	}
	else
	{
		return 2;
	}
	
	return 0x00; 	//ok
} 

//����ͼ���������
//width:���ͼ����,<=320
//height:���ͼ��߶�,<=240
//mode:0��QVGA���ģʽ��1��VGA���ģʽ
//QVGAģʽ���ӷ�Χ�㵫���ﲻ�Ǻ�������VGAģʽ���ӷ�ΧС��������
void OV7725_Window_Set(u16 width,u16 height,u8 mode)
{
	u8 raw,temp;
	u16 sx,sy;	
	if(mode)
	{
		sx=(640-width)/2;
		sy=(480-height)/2;
		SCCB_WR_Reg(COM7,0x06);		//����ΪVGAģʽ
		SCCB_WR_Reg(HSTART,0x23); 	//ˮƽ��ʼλ��
		SCCB_WR_Reg(HSIZE,0xA0); 	//ˮƽ�ߴ�
		SCCB_WR_Reg(VSTRT,0x07); 	//��ֱ��ʼλ��
		SCCB_WR_Reg(VSIZE,0xF0); 	//��ֱ�ߴ�
		SCCB_WR_Reg(HREF,0x00);
		SCCB_WR_Reg(HOutSize,0xA0); //����ߴ�
		SCCB_WR_Reg(VOutSize,0xF0); //����ߴ�
	}
	else
	{
		sx=(320-width)/2;
		sy=(240-height)/2;
		SCCB_WR_Reg(COM7,0x46);		//����ΪQVGAģʽ
		SCCB_WR_Reg(HSTART,0x3f); 	//ˮƽ��ʼλ��
		SCCB_WR_Reg(HSIZE, 0x50); 	//ˮƽ�ߴ�
		SCCB_WR_Reg(VSTRT, 0x03); 	//��ֱ��ʼλ��
		SCCB_WR_Reg(VSIZE, 0x78); 	//��ֱ�ߴ�
		SCCB_WR_Reg(HREF,  0x00);
		SCCB_WR_Reg(HOutSize,0x50);	//����ߴ�
		SCCB_WR_Reg(VOutSize,0x78); //����ߴ�
	}
	raw=SCCB_RD_Reg(HSTART);
	temp=raw+(sx>>2);//sx��8λ����HSTART,��2λ����HREF[5:4]
	SCCB_WR_Reg(HSTART,temp);
	SCCB_WR_Reg(HSIZE,width>>2);//width��8λ����HSIZE,��2λ����HREF[1:0]
	
	raw=SCCB_RD_Reg(VSTRT);
	temp=raw+(sy>>1);//sy��8λ����VSTRT,��1λ����HREF[6]
	SCCB_WR_Reg(VSTRT,temp);
	SCCB_WR_Reg(VSIZE,height>>1);//height��8λ����VSIZE,��1λ����HREF[2]
	
	raw=SCCB_RD_Reg(HREF);
	temp=((sy&0x01)<<6)|((sx&0x03)<<4)|((height&0x01)<<2)|(width&0x03)|raw;
	SCCB_WR_Reg(HREF,temp);
	
	SCCB_WR_Reg(HOutSize,width>>2);
	SCCB_WR_Reg(VOutSize,height>>1);
	
	SCCB_RD_Reg(EXHCH);	
	temp = (raw|(width&0x03)|((height&0x01)<<2));	
	SCCB_WR_Reg(EXHCH,temp);	
}












