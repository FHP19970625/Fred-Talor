
#include "include.h" 
 uint16  BATTVOL; 
 uint16 M1pmw,M2pmw,M3pmw,M4pmw;
extern struct _PID PID_US100;//��������pid����
u8 sumtxdbuf,TXDBUF,txdi;
u8 rx_buf[32];	
u8 tx_buf[32]={0};	
int main(void)
{     

     Gyro_OFFEST(); 
	sensor.acc.CALIBRATE=0;	
   	IAC_Init();     //�ӿ��Լ�����ĳ�ʼ��
	  Sensor_Init();  //������ ��ʼ��      
	  paramLoad();//pid��������
	  State_Display();//OLED������ʾ
	  ALGH_set();   //���������γ�
	  EnTIM3();       //����ʱ��				 
	
    while(1)
	  {    
      /* GPS������� */
//    nmea_decode_test();

			BATTDispaly(); //��ѹ��ʾ
	  }
}
/******************* (C) COPYRIGHT 2014 BEYOND *****END OF FILE************/

