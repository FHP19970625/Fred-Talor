#include "include.h"
extern u8 rx_buf[32];	
extern struct _PID PID_US100;//超声波的pid参数
#define USART_len 100
#define NRF_len 100
	// The data of pitch
#define	ctrl_pitch_shell_kp_ADD  0x0807BB08//2;    //5
#define	ctrl_pitch_shell_ki_ADD  0x0807BB0C// 0.01;
#define	ctrl_pitch_shell_kd_ADD  0x0807BB10// 0.6;    //2
	
#define	ctrl_pitch_core_kp_ADD  0x0807BB14// 1.8;   //1.5
#define ctrl_pitch_core_ki_ADD  0x0807BB18
#define	ctrl_pitch_core_kd_ADD  0x0807BB1C// 0.22;  //0.16
	
	//The data of roll
#define	ctrl_roll_shell_kp_ADD  0x0807BB20// 2;
#define	ctrl_roll_shell_ki_ADD  0x0807BB24// 0.01;
#define	ctrl_roll_shell_kd_ADD  0x0807BB28// 0.6;

#define	ctrl_roll_core_kp_ADD  0x0807BB2C// 1.8;
#define ctrl_roll_core_ki_ADD  0x0807BB30
#define	ctrl_roll_core_kd_ADD  0x0807BB34//0.22;
	
	//The data of yaw
#define	ctrl_yaw_shell_kp_ADD  0x0807BB38// 5;
#define ctrl_yaw_shell_ki_ADD  0x0807BB3C
#define	ctrl_yaw_shell_kd_ADD  0x0807BB40// 0.13;
	
#define	ctrl_yaw_core_kp_ADD   0x0807BB44//1.8;
#define ctrl_yaw_core_ki_ADD   0x0807BB48
#define	ctrl_yaw_core_kd_ADD   0x0807BB4C// 0.1;
//u16 USART_RX_STA=0;
u8 commad=0;//1=读PID,2=写PID，3=未定义
u8 sendms=0;
//u8 USART_RX_BUF[USART_len];//usart3串口接收到的数据组		
u8 USART_TX_BUF[USART_len];//usart3串口发送到的数据组
u8 NRF_RX_BUF[NRF_len];//nrf接收到的数据组		
u8 NRF_TX_BUF[NRF_len];//nrf发送到的数据组
uint16_t VirtAddVarTab[NumbOfVar] = {0xAA00, 0xAA01, 0xAA02};
unsigned char floatTURN2char1000x_char[2];

float fanhuifudianshu(char a ,char b)
{
	float g,s,ba,q;
	u16 datam=0;
							datam=(256*a)+b;
						    g=datam%10;
						    g/=1000;
						    datam/=10;
						    s=datam%10;
						    s/=100;
						    datam/=10;
						    ba=datam%10;
						    ba/=10;
						    datam/=10;
						    q=datam%10;
	return g+s+ba+q;
}
void PID_shell_SAVE(void)
{
	float tempdata1;
	u16 byte1,byte2;
	
	tempdata1=ctrl.pitch.shell.kp;
	byte1=((u16 *)&tempdata1)[0];//拆开
  byte2=((u16 *)&tempdata1)[1];
	STMFLASH_Write(ctrl_pitch_shell_kp_ADD, &byte1,1);
	STMFLASH_Write((ctrl_pitch_shell_kp_ADD+2), &byte2,1);
	
	
//	STMFLASH_Write(ctrl_pitch_shell_ki_ADD, &(ctrl.pitch.shell.ki),2);
	tempdata1=ctrl.pitch.shell.ki;
	byte1=((u16 *)&tempdata1)[0];//拆开
  byte2=((u16 *)&tempdata1)[1];
	STMFLASH_Write(ctrl_pitch_shell_ki_ADD, &byte1,1);
	STMFLASH_Write((ctrl_pitch_shell_ki_ADD+2), &byte2,1);
	
//	STMFLASH_Write(ctrl_pitch_shell_kd_ADD, &(ctrl.pitch.shell.kd),2);
	tempdata1=ctrl.pitch.shell.kd;
	byte1=((u16 *)&tempdata1)[0];//拆开
  byte2=((u16 *)&tempdata1)[1];
	STMFLASH_Write(ctrl_pitch_shell_kd_ADD, &byte1,1);
	STMFLASH_Write((ctrl_pitch_shell_kd_ADD+2), &byte2,1);
	
	//The data of roll
//	ctrl.roll.shell.kp = 2;
//	ctrl.roll.shell.ki = 0.01;
//ctrl.roll.shell.kd = 0.6;

	tempdata1=ctrl.roll.shell.kp;
	byte1=((u16 *)&tempdata1)[0];//拆开
  byte2=((u16 *)&tempdata1)[1];
	STMFLASH_Write(ctrl_roll_shell_kp_ADD, &byte1,1);
	STMFLASH_Write((ctrl_roll_shell_kp_ADD+2), &byte2,1);
	
	tempdata1=ctrl.roll.shell.ki;
	byte1=((u16 *)&tempdata1)[0];//拆开
  byte2=((u16 *)&tempdata1)[1];
	STMFLASH_Write(ctrl_roll_shell_ki_ADD, &byte1,1);
	STMFLASH_Write((ctrl_roll_shell_ki_ADD+2), &byte2,1);
	
	tempdata1=ctrl.roll.shell.kd;
	byte1=((u16 *)&tempdata1)[0];//拆开
  byte2=((u16 *)&tempdata1)[1];
	STMFLASH_Write(ctrl_roll_shell_kd_ADD, &byte1,1);
	STMFLASH_Write((ctrl_roll_shell_kd_ADD+2), &byte2,1);


	
	//The data of yaw
//	ctrl.yaw.shell.kp = 5;
//	ctrl.yaw.shell.kd = 0.13;

	tempdata1=ctrl.yaw.shell.kp;
	byte1=((u16 *)&tempdata1)[0];//拆开
  byte2=((u16 *)&tempdata1)[1];
	STMFLASH_Write(ctrl_yaw_shell_kp_ADD, &byte1,1);
	STMFLASH_Write((ctrl_yaw_shell_kp_ADD+2), &byte2,1);
	
	tempdata1=ctrl.yaw.shell.ki;
	byte1=((u16 *)&tempdata1)[0];//拆开
  byte2=((u16 *)&tempdata1)[1];
	STMFLASH_Write(ctrl_yaw_shell_ki_ADD, &byte1,1);
	STMFLASH_Write((ctrl_yaw_shell_ki_ADD+2), &byte2,1);
	
	tempdata1=ctrl.yaw.shell.kd;
	byte1=((u16 *)&tempdata1)[0];//拆开
  byte2=((u16 *)&tempdata1)[1];
	STMFLASH_Write(ctrl_yaw_shell_kd_ADD, &byte1,1);
	STMFLASH_Write((ctrl_yaw_shell_kd_ADD+2), &byte2,1);
	
}	
	
void PID_roll_SAVE(void)
{
	float tempdata2;
	u16 byte1,byte2;
	
//	ctrl.pitch.core.kp = 1.8;   //1.5
//	ctrl.pitch.core.kd = 0.22;  //0.16
//	STMFLASH_Write(ctrl_pitch_core_kp_ADD, &(ctrl.pitch.core.kp),2);
	tempdata2=ctrl.pitch.core.kp;
	byte1=((u16 *)&tempdata2)[0];//拆开
  byte2=((u16 *)&tempdata2)[1];
	STMFLASH_Write(ctrl_pitch_core_kp_ADD, &byte1,1);
	STMFLASH_Write((ctrl_pitch_core_kp_ADD+2), &byte2,1);
	
	tempdata2=ctrl.pitch.core.ki;
	byte1=((u16 *)&tempdata2)[0];//拆开
  byte2=((u16 *)&tempdata2)[1];
	STMFLASH_Write(ctrl_pitch_core_ki_ADD, &byte1,1);
	STMFLASH_Write((ctrl_pitch_core_ki_ADD+2), &byte2,1);
	
	
//	STMFLASH_Write(ctrl_pitch_core_kd_ADD, &(ctrl.pitch.core.kd),2);
	tempdata2=ctrl.pitch.core.kd;
	byte1=((u16 *)&tempdata2)[0];//拆开
  byte2=((u16 *)&tempdata2)[1];
	STMFLASH_Write(ctrl_pitch_core_kd_ADD, &byte1,1);
	STMFLASH_Write((ctrl_pitch_core_kd_ADD+2), &byte2,1);
//	ctrl.roll.core.kp = 1.8;
//	ctrl.roll.core.kd = 0.22;

  tempdata2=ctrl.roll.core.kp;
	byte1=((u16 *)&tempdata2)[0];//拆开
  byte2=((u16 *)&tempdata2)[1];
	STMFLASH_Write(ctrl_roll_core_kp_ADD, &byte1,1);
	STMFLASH_Write((ctrl_roll_core_kp_ADD+2), &byte2,1);
	
	 tempdata2=ctrl.roll.core.ki;
	byte1=((u16 *)&tempdata2)[0];//拆开
  byte2=((u16 *)&tempdata2)[1];
	STMFLASH_Write(ctrl_roll_core_ki_ADD, &byte1,1);
	STMFLASH_Write((ctrl_roll_core_ki_ADD+2), &byte2,1);
	
	tempdata2=ctrl.roll.core.kd;
	byte1=((u16 *)&tempdata2)[0];//拆开
  byte2=((u16 *)&tempdata2)[1];
	STMFLASH_Write(ctrl_roll_core_kd_ADD, &byte1,1);
	STMFLASH_Write((ctrl_roll_core_kd_ADD+2), &byte2,1);
	
	
	//ctrl.yaw.core.kp = 1.8;
//	ctrl.yaw.core.kd = 0.1;

	tempdata2=ctrl.yaw.core.kp;
	byte1=((u16 *)&tempdata2)[0];//拆开
  byte2=((u16 *)&tempdata2)[1];
	STMFLASH_Write(ctrl_yaw_core_kp_ADD, &byte1,1);
	STMFLASH_Write((ctrl_yaw_core_kp_ADD+2), &byte2,1);
	
	tempdata2=ctrl.yaw.core.ki;
	byte1=((u16 *)&tempdata2)[0];//拆开
  byte2=((u16 *)&tempdata2)[1];
	STMFLASH_Write(ctrl_yaw_core_ki_ADD, &byte1,1);
	STMFLASH_Write((ctrl_yaw_core_ki_ADD+2), &byte2,1);
	
	tempdata2=ctrl.yaw.core.kd;
	byte1=((u16 *)&tempdata2)[0];//拆开
  byte2=((u16 *)&tempdata2)[1];
	STMFLASH_Write(ctrl_yaw_core_kd_ADD, &byte1,1);
	STMFLASH_Write((ctrl_yaw_core_kd_ADD+2), &byte2,1);

}
void PID_shell_Read(void)
{
		float tempdata1;
	  u16 byte1[2];//byte2,byte3,byte4;
	// The data of pitch
//	ctrl.pitch.shell.kp = 2;    //5
//	ctrl.pitch.shell.ki = 0.01;
//	ctrl.pitch.shell.kd = 0.6;    //2
	
// 	 ((u8 *)&tempdata)[0]=byte1;//合并
//   ((u8 *)&tempdata)[1]=byte2;
//   ((u8 *)&tempdata)[2]=byte3;
//   ((u8 *)&tempdata)[3]=byte4;
	
	STMFLASH_Read(ctrl_pitch_shell_kp_ADD, &byte1[0],2);
	((u16 *)&tempdata1)[0]=byte1[0];//合并
  ((u16 *)&tempdata1)[1]=byte1[1];
	ctrl.pitch.shell.kp=tempdata1;
	
	STMFLASH_Read(ctrl_pitch_shell_ki_ADD,  &byte1[0],2);
	((u16 *)&tempdata1)[0]=byte1[0];//合并
  ((u16 *)&tempdata1)[1]=byte1[1];
	ctrl.pitch.shell.ki=tempdata1;
	
	STMFLASH_Read(ctrl_pitch_shell_kd_ADD, &byte1[0],2);
	((u16 *)&tempdata1)[0]=byte1[0];//合并
  ((u16 *)&tempdata1)[1]=byte1[1];
	ctrl.pitch.shell.kd=tempdata1;
	
		//The data of roll
//	ctrl.roll.shell.kp = 2;
//	ctrl.roll.shell.ki = 0.01;
//ctrl.roll.shell.kd = 0.6;
  STMFLASH_Read(ctrl_roll_shell_kp_ADD, &byte1[0],2);
	((u16 *)&tempdata1)[0]=byte1[0];//合并
  ((u16 *)&tempdata1)[1]=byte1[1];
	ctrl.roll.shell.kp=tempdata1;
	
	STMFLASH_Read(ctrl_roll_shell_ki_ADD, &byte1[0],2);
	((u16 *)&tempdata1)[0]=byte1[0];//合并
  ((u16 *)&tempdata1)[1]=byte1[1];
	ctrl.roll.shell.ki=tempdata1;
	
	STMFLASH_Read(ctrl_roll_shell_kd_ADD, &byte1[0],2);
	((u16 *)&tempdata1)[0]=byte1[0];//合并
  ((u16 *)&tempdata1)[1]=byte1[1];
	ctrl.roll.shell.kd=tempdata1;
	
	//The data of yaw
//	ctrl.yaw.shell.kp = 5;
//	ctrl.yaw.shell.kd = 0.13;
	STMFLASH_Read(ctrl_yaw_shell_kp_ADD, &byte1[0],2);
	((u16 *)&tempdata1)[0]=byte1[0];//合并
  ((u16 *)&tempdata1)[1]=byte1[1];
	ctrl.yaw.shell.kp=tempdata1;
	
	STMFLASH_Read(ctrl_yaw_shell_ki_ADD, &byte1[0],2);
	((u16 *)&tempdata1)[0]=byte1[0];//合并
  ((u16 *)&tempdata1)[1]=byte1[1];
	ctrl.yaw.shell.ki=tempdata1;
	
	STMFLASH_Read(ctrl_yaw_shell_kd_ADD, &byte1[0],2);
	((u16 *)&tempdata1)[0]=byte1[0];//合并
  ((u16 *)&tempdata1)[1]=byte1[1];
	ctrl.yaw.shell.kd=tempdata1;
	
}	
void PID_roll_Read(void)
{
	  float tempdata2;
	  u16 byte1[2];//byte2,byte3,byte4;
//	ctrl.pitch.core.kp = 1.8;   //1.5
//	ctrl.pitch.core.kd = 0.22;  //0.16
	STMFLASH_Read(ctrl_pitch_core_kp_ADD, &byte1[0],2);
	((u16 *)&tempdata2)[0]=byte1[0];//合并
  ((u16 *)&tempdata2)[1]=byte1[1];
	ctrl.pitch.core.kp=tempdata2;
	
		STMFLASH_Read(ctrl_pitch_core_ki_ADD, &byte1[0],2);
	((u16 *)&tempdata2)[0]=byte1[0];//合并
  ((u16 *)&tempdata2)[1]=byte1[1];
	ctrl.pitch.core.ki=tempdata2;
	
	STMFLASH_Read(ctrl_pitch_core_kd_ADD, &byte1[0],2);
	((u16 *)&tempdata2)[0]=byte1[0];//合并
  ((u16 *)&tempdata2)[1]=byte1[1];
	ctrl.pitch.core.kd=tempdata2;
	

//	ctrl.roll.core.kp = 1.8;
//	ctrl.roll.core.kd = 0.22;
	STMFLASH_Read(ctrl_roll_core_kp_ADD, &byte1[0],2);
	((u16 *)&tempdata2)[0]=byte1[0];//合并
  ((u16 *)&tempdata2)[1]=byte1[1];
	ctrl.roll.core.kp=tempdata2;
	
	STMFLASH_Read(ctrl_roll_core_ki_ADD, &byte1[0],2);
	((u16 *)&tempdata2)[0]=byte1[0];//合并
  ((u16 *)&tempdata2)[1]=byte1[1];
	ctrl.roll.core.ki=tempdata2;
	
	STMFLASH_Read(ctrl_roll_core_kd_ADD, &byte1[0],2);
	((u16 *)&tempdata2)[0]=byte1[0];//合并
  ((u16 *)&tempdata2)[1]=byte1[1];
	ctrl.roll.core.kd=tempdata2;
	
	
	
	//ctrl.yaw.core.kp = 1.8;
//	ctrl.yaw.core.kd = 0.1;
	STMFLASH_Read(ctrl_yaw_core_kp_ADD, &byte1[0],2);
	((u16 *)&tempdata2)[0]=byte1[0];//合并
  ((u16 *)&tempdata2)[1]=byte1[1];
	ctrl.yaw.core.kp=tempdata2;
	
	STMFLASH_Read(ctrl_yaw_core_ki_ADD, &byte1[0],2);
	((u16 *)&tempdata2)[0]=byte1[0];//合并
  ((u16 *)&tempdata2)[1]=byte1[1];
	ctrl.yaw.core.ki=tempdata2;
	
	STMFLASH_Read(ctrl_yaw_core_kd_ADD, &byte1[0],2);
	((u16 *)&tempdata2)[0]=byte1[0];//合并
  ((u16 *)&tempdata2)[1]=byte1[1];
	ctrl.yaw.core.kd=tempdata2;
	
}		
u16 US_100_DATA;	
uint8_t ReceiveDate;								//定义一个变量存放接收的数据
u8 US_100_DATA_H;
u8 US_100_DATA_L;
extern int USE_US100_IS;
static vu16 Alt_Temp=0,Alt_Cunt=0,US100_Alt_Temp=0; 
 float 
US100_Alt_V,Alt_CuntTmep1=0,Alt_CuntTmep2=0,Alt_Last=0,US100_Alt_Last=0,Alt_V_CuntTmep1=0,Alt_V_CuntTmep2=0,US100_Alt; 
static u8 count_usart2=0;
//void USART2_IRQHandler(void)//30ms
//{

//       
//	if(!(USART_GetITStatus(USART2,USART_IT_RXNE))); 	//读取接收中断标志位USART_IT_RXNE 
//														//USART_FLAG_RXNE:接收数据寄存器非空标志位 
//														//1：忙状态  0：空闲(没收到数据，等待。。。)
//	{
//		count_usart2++;
//		USART_ClearITPendingBit(USART2,USART_IT_RXNE);	//清楚中断标志位
//		ReceiveDate=USART_ReceiveData(USART2);			//接收字符存入数组

//		if(count_usart2==1) 
//			Alt_Temp=ReceiveDate;//接收高字节
//		if(count_usart2==2)
//		{
//     count_usart2=0;	
//			US_100_DATA_L=ReceiveDate;
//			Alt_Temp=Alt_Temp<<8;//移位到高位
//		US100_Alt_Temp=(Alt_Temp|US_100_DATA_L);//变成16位的数据
//		   if(US100_Alt_Temp>0x1194)     //极限4500mm 
//{ 
//      US100_Alt_Temp=Alt_Last; 
//} 
//else   
//{ 
//    Alt_Last=US100_Alt_Temp; 
//}
//Alt_CuntTmep2=Alt_CuntTmep1;        //滑动平均滤波
//Alt_CuntTmep1=US100_Alt_Temp*((float)cos(angle.pitch/57.295779f))*((float)cos(angle.roll/
//57.295779f));        //姿态补偿
//US100_Alt=((Alt_CuntTmep1+Alt_CuntTmep2)/2)/1000;  //除以1000转化为M
// 
//Alt_V_CuntTmep2=Alt_V_CuntTmep1;//滑动平均滤波
//Alt_V_CuntTmep1=(US100_Alt-US100_Alt_Last)/ 0.05f;   //除以0.05s获得速度单位：m/S 
// 
//US100_Alt_V= (Alt_V_CuntTmep1+Alt_V_CuntTmep2)/2;   
//US100_Alt_Last=US100_Alt; 
//	  }
//	}											 
//}   

//void USART3_IRQHandler(void)                	//串口3中断服务程序
//	{
//	u8 Res,i,datacheck;
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//		{
//		  Res =USART_ReceiveData(USART3);//(USART1->DR);	//读取接收到的数据
//		  if(Res==0x8A&&USART_RX_STA==0&&commad==0) 
//		  {
//		  	USART_RX_STA=60;//接收到数据头，无条件接收剩下60个字节
//		  	USART_RX_BUF[0]=Res;
//		  }
//		else if(USART_RX_STA==1&&commad==0)//接收到最后一个
//			{
//				USART_RX_BUF[61-USART_RX_STA]=Res;
//				if(USART_RX_BUF[0]==0x8A)
//				if(USART_RX_BUF[1]==0x6D)
//				if(USART_RX_BUF[2]==0x1C)
//				if(USART_RX_BUF[3]==0xAD)
//				if(!(commad))
//				 {
//					 datacheck=0;
//					 for(i=0;i<60;i++)datacheck+=USART_RX_BUF[i];
//					 if(datacheck==USART_RX_BUF[60])commad=1;//读PID命令 
//					 sendms=5;//停顿20*5=100ms后再发数据
//				 }

//				if(USART_RX_BUF[0]==0x8A)
//				if(USART_RX_BUF[1]==0x6B)
//				if(USART_RX_BUF[2]==0x1C)
//				if(USART_RX_BUF[3]==0xAB) 
//				if(!(commad))
//				 {
//					 datacheck=0;
//					 for(i=0;i<60;i++)datacheck+=USART_RX_BUF[i];
//					 if(datacheck==USART_RX_BUF[60])
//					 {
//						    //写PID命令 

//						 ctrl.roll.shell.kp=fanhuifudianshu(USART_RX_BUF[6],USART_RX_BUF[7]);
//             ctrl.roll.shell.ki=fanhuifudianshu(USART_RX_BUF[10],USART_RX_BUF[11]);
//						 ctrl.roll.shell.kd=fanhuifudianshu(USART_RX_BUF[14],USART_RX_BUF[15]);
//						 ctrl.pitch.shell.kp=fanhuifudianshu(USART_RX_BUF[18],USART_RX_BUF[19]);
//						 ctrl.pitch.shell.ki=fanhuifudianshu(USART_RX_BUF[22],USART_RX_BUF[23]);
//						 ctrl.pitch.shell.kd=fanhuifudianshu(USART_RX_BUF[26],USART_RX_BUF[27]);
//						 ctrl.yaw.shell.kp=fanhuifudianshu(USART_RX_BUF[30],USART_RX_BUF[31]);
//						 ctrl.yaw.shell.kd=fanhuifudianshu(USART_RX_BUF[34],USART_RX_BUF[35]);
//						 ctrl.roll.core.kp=fanhuifudianshu(USART_RX_BUF[38],USART_RX_BUF[39]);
//						 ctrl.roll.core.kd=fanhuifudianshu(USART_RX_BUF[42],USART_RX_BUF[43]);
//						 ctrl.pitch.core.kp=fanhuifudianshu(USART_RX_BUF[46],USART_RX_BUF[47]);
//						 ctrl.pitch.core.kd=fanhuifudianshu(USART_RX_BUF[50],USART_RX_BUF[51]);
////						 ctrl.yaw.core.kp=fanhuifudianshu(USART_RX_BUF[54],USART_RX_BUF[55]);
////						 ctrl.yaw.core.kd=fanhuifudianshu(USART_RX_BUF[58],USART_RX_BUF[59]);
//						 PID_US100.P=fanhuifudianshu(USART_RX_BUF[54],USART_RX_BUF[55])*1000;//换成超声波的数据流
//						 PID_US100.D=fanhuifudianshu(USART_RX_BUF[58],USART_RX_BUF[59])*1000;
//								
//					      PID_SAVE();//保存参数
//								PID_Read();//读出参数发给上位机
//								commad=1;//启动发送
//					 }
//				 }	 
//				USART_RX_STA=0;
//			}
//		else if(USART_RX_STA&&commad==0)//接收未完成
//			{
//        USART_RX_BUF[61-USART_RX_STA]=Res;
//			  USART_RX_STA--;
//		  }
//	}
//}

static void EE_READ_ACC_OFFSET(void)
{
	EE_ReadVariable(VirtAddVarTab[0], &sensor.acc.quiet.x);
	EE_ReadVariable(VirtAddVarTab[1], &sensor.acc.quiet.y);
	EE_ReadVariable(VirtAddVarTab[2], &sensor.acc.quiet.z);
}

void EE_SAVE_ACC_OFFSET(void)
{
  EE_WriteVariable(VirtAddVarTab[0],sensor.acc.quiet.x);
  EE_WriteVariable(VirtAddVarTab[1],sensor.acc.quiet.y);
	EE_WriteVariable(VirtAddVarTab[2],sensor.acc.quiet.z);
}	

void floatTURN2char1000x(float data)//浮点数转为1字节数
{
	u16 datatempf;
 if(data<10) data*=1000;
	{
		datatempf=data;
		floatTURN2char1000x_char[1]=datatempf;//
	  floatTURN2char1000x_char[0]=datatempf>>8;//
	}
}
//**************************************************************************
//参数加载
//**************************************************************************
void	paramLoad(void)
{
	EE_READ_ACC_OFFSET(); //读取加速度零偏
	Gyro_OFFEST();  //采集陀螺仪零偏
	
//	// The data of pitch
//	ctrl.pitch.shell.kp = 4.1;    //5 外环
//	ctrl.pitch.shell.ki = 0.02;
//	ctrl.pitch.shell.kd = 0.21;    //2
//	
//	ctrl.pitch.core.kp = 1.2;   //1.5  内环
//	ctrl.pitch.core.kd = 0.02;  //0.16
//	
//	//The data of roll
//	ctrl.roll.shell.kp = 4.13;
//	ctrl.roll.shell.ki = 0.02;
//	ctrl.roll.shell.kd = 0.21;

//	ctrl.roll.core.kp = 1.23;
//	ctrl.roll.core.kd = 0.02;
//	
//	//The data of yaw
//	ctrl.yaw.shell.kp = 4.55;
//	ctrl.yaw.shell.kd = 0.13;
//	
//	ctrl.yaw.core.kp = 1.8;
//	ctrl.yaw.core.kd = 0.1;
//	//The data of yaw
////	ctrl.yaw.shell.kp = 0;
////	ctrl.yaw.shell.kd = 0.0;
////	
////	ctrl.yaw.core.kp = 0;
////	ctrl.yaw.core.kd = 0;
//	
//	//超声波的油门限高PID参数
//	PID_US100.P=210;
//	PID_US100.D=60;
//	PID_US100.I=0;


//	//limit for the max increment
//	ctrl.pitch.shell.increment_max = 200;
//	ctrl.roll.shell.increment_max = 200;
//	
//	ctrl.ctrlRate = 0;

//	Rc_Data.pitch_offset = 1530;
//	Rc_Data.roll_offset = 1530;
//	Rc_Data.yaw_offset = 1530;
}
void Receive_NRF_Data(void)
{
		NRF24L01_RX_Mode();		  
		if(NRF24L01_RxPacket(rx_buf)==0)//一旦接收到信息,则显示出来.
			{	
		 if(commad==0)
			{
				if(rx_buf[1]==0x8A)
				if(rx_buf[2]==0x6D)
				if(rx_buf[3]==0x1C)
				if(rx_buf[4]==0xAD)
				if(!(commad))
				 {
//					 datacheck=0;
//					 for(i=0;i<60;i++)datacheck+=rx_buf[i];
//					 if(datacheck==rx_buf[60])
						 commad=1;//读PID命令 
					 sendms=5;//停顿20*5=100ms后再发数据
				 }
			 }
			}  
}
