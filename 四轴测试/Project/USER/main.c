
#include "include.h" 
 uint16  BATTVOL; 
 uint16 M1pmw,M2pmw,M3pmw,M4pmw;
extern struct _PID PID_US100;//超声波的pid参数
u8 sumtxdbuf,TXDBUF,txdi;
u8 rx_buf[32];	
u8 tx_buf[32]={0};	
int main(void)
{     

     Gyro_OFFEST(); 
	sensor.acc.CALIBRATE=0;	
   	IAC_Init();     //接口以及外设的初始化
	  Sensor_Init();  //传感器 初始化      
	  paramLoad();//pid参数加载
	  State_Display();//OLED数据显示
	  ALGH_set();   //设置油门形成
	  EnTIM3();       //开定时器				 
	
    while(1)
	  {    
      /* GPS解码测试 */
//    nmea_decode_test();

			BATTDispaly(); //电压显示
	  }
}
/******************* (C) COPYRIGHT 2014 BEYOND *****END OF FILE************/

