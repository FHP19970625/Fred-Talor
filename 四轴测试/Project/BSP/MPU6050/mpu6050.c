#include "include.h"

struct _sensor sensor;	
u8		 mpu6050_buffer[14];					//iic读取后存放数据 	

u8		ACC_OFFSET_OK = 0;


extern vs16 Moto_duty[4];

//**************************************
//初始化MPU6050
//**************************************
u8 InitMPU6050(void)
{
	u8 date;
	date = Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);  	//解除休眠状态
	date += Single_Write(MPU6050_ADDRESS, SMPLRT_DIV, 0x07);     
	date += Single_Write(MPU6050_ADDRESS, cONFIG, 0x03);         //低通滤波
	date += Single_Write(MPU6050_ADDRESS, GYRO_CONFIG, 0x10);    //陀螺仪量程 +-1000
	date += Single_Write(MPU6050_ADDRESS, ACCEL_CONFIG, 0x09);   //加速度量程 +-4G
	date +=	Single_Write(MPU6050_ADDRESS,MPU_INTBP_CFG_REG,0x42);    //使能旁路IIC
	date += Single_Write(MPU6050_ADDRESS,MPU_USER_CTRL_REG,0x40);
	return date;
}
//**************************实现函数********************************************
//将iic读取到得数据分拆,放入相应寄存器,更新MPU6050_Last
//******************************************************************************
void MPU6050_Read(void)
{
	mpu6050_buffer[0]=Single_Read(MPU6050_ADDRESS, 0x3B);
	mpu6050_buffer[1]=Single_Read(MPU6050_ADDRESS, 0x3C);
	mpu6050_buffer[2]=Single_Read(MPU6050_ADDRESS, 0x3D);
	mpu6050_buffer[3]=Single_Read(MPU6050_ADDRESS, 0x3E);
	mpu6050_buffer[4]=Single_Read(MPU6050_ADDRESS, 0x3F);
	mpu6050_buffer[5]=Single_Read(MPU6050_ADDRESS, 0x40);
	mpu6050_buffer[8]=Single_Read(MPU6050_ADDRESS, 0x43);
	mpu6050_buffer[9]=Single_Read(MPU6050_ADDRESS, 0x44);
	mpu6050_buffer[10]=Single_Read(MPU6050_ADDRESS, 0x45);
	mpu6050_buffer[11]=Single_Read(MPU6050_ADDRESS, 0x46);
	mpu6050_buffer[12]=Single_Read(MPU6050_ADDRESS, 0x47);
	mpu6050_buffer[13]=Single_Read(MPU6050_ADDRESS, 0x48);
	
}
/**************************实现函数********************************************
//将iic读取到得数据分拆,放入相应寄存器
*******************************************************************************/
void MPU6050_Dataanl(void)
{
	MPU6050_Read();
	
	sensor.acc.origin.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - sensor.acc.quiet.x;
	sensor.acc.origin.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - sensor.acc.quiet.y;
	sensor.acc.origin.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]);

	sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
  
	sensor.gyro.radian.x = sensor.gyro.origin.x * Gyro_Gr - sensor.gyro.quiet.x * Gyro_Gr;
	sensor.gyro.radian.y = sensor.gyro.origin.y * Gyro_Gr - sensor.gyro.quiet.y * Gyro_Gr;
	sensor.gyro.radian.z = sensor.gyro.origin.z * Gyro_Gr - sensor.gyro.quiet.z * Gyro_Gr;


////////////////////////////////////////////////////
//    	The calibration  of  gyro and  acc        //
////////////////////////////////////////////////////	
 	if(sensor.acc.CALIBRATE==1)
	{
		MPU_Acc_Offset();

		sensor.acc.CALIBRATE=0;
	}
 
}
void MPU_Acc_Offset(void)
{
	u16 cnt=2000;
  sensor.acc.temp.x=0;
	sensor.acc.temp.y=0;
	sensor.acc.temp.z=0;

	 sensor.gyro.averag.x=0;    //零点偏移清零
	 sensor.gyro.averag.y=0;  
	 sensor.gyro.averag.z=0;
	 while(cnt--)       //循环采集2000次   求平均
	 {
		MPU6050_Read();


		sensor.acc.origin.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]);
		sensor.acc.origin.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]);
		sensor.acc.origin.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]);


		sensor.acc.temp.x+= sensor.acc.origin.x;
		sensor.acc.temp.y+= sensor.acc.origin.y;
		sensor.acc.temp.z+= sensor.acc.origin.z;
   	}
	cnt=2000;


	sensor.acc.quiet.x=(sensor.acc.temp.x/cnt);
	sensor.acc.quiet.y=(sensor.acc.temp.y/cnt);
	sensor.acc.quiet.z=(sensor.acc.temp.z/cnt);
	 


	 
}	
/**************************实现函数********************************************
//陀螺仪零点校准
*******************************************************************************/

void Gyro_OFFEST(void)
{
   int cnt_g=2000;
	 int32_t  tempgx=0,tempgy=0,tempgz=0;
	 sensor.gyro.averag.x=0;    //零点偏移清零
	 sensor.gyro.averag.y=0;  
	 sensor.gyro.averag.z=0;
	 while(cnt_g--)       //循环采集2000次   求平均
	 {
		  MPU6050_Read();
		 
		  sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	    sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	    sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
      tempgx+= sensor.gyro.origin.x;
			tempgy+= sensor.gyro.origin.y;
			tempgz+= sensor.gyro.origin.z;
   }
	 sensor.gyro.quiet.x=tempgx/2000;
	 sensor.gyro.quiet.y=tempgy/2000;
	 sensor.gyro.quiet.z=tempgz/2000;
	  
}





//extern float qa0, qa1, qa2, qa3;



//void UART1_ReportIMU(void)
//{
//  int16_t ax,ay,az,gx,gy,gz;
//  int16_t hx,hy,hz;
//  int16_t yaw,pitch,roll;
//  uint8_t temp;
//	char ctemp;
//	UART_Put_Char(0x88); 
//	UART_Put_Char(0xAF);  
//	UART_Put_Char(0x1C);  

//	ax=sensor.acc.origin.x;

//	ctemp=ax>>8;
//	UART_Put_Char(ctemp); 
//	temp+=ctemp;
//	ctemp=ax;
//	UART_Put_Char(ctemp); 
//	temp+=sensor.acc.origin.y;

//	ay=angle.roll*110;

//	ctemp=ay>>8;
//	UART_Put_Char(ctemp); 
//	temp+=ctemp;
//	ctemp=ay;
//	UART_Put_Char(ctemp); 
//	temp+=ctemp;

//	az=qa0*10000;

//	ctemp=az>>8;
//	UART_Put_Char(ctemp);
//	temp+=ctemp;
//	ctemp=az;
//	UART_Put_Char(ctemp);
//	temp+=ctemp;

//	gx=qa1*10000;

//	ctemp=gx>>8;
//	UART_Put_Char(ctemp);
//	temp+=ctemp;
//	ctemp=gx;
//	UART_Put_Char(ctemp);
//	temp+=ctemp;

//	gy=qa2*10000;

//	ctemp=gy>>8;
//	UART_Put_Char(ctemp);
//	temp+=ctemp;
//	ctemp=gy;
//	UART_Put_Char(ctemp);
//	temp+=ctemp;

//  gz=qa3*10000;

//	ctemp=gz>>8;
//	UART_Put_Char(ctemp); 
//	temp+=ctemp;
//	ctemp=gz;
//	UART_Put_Char(ctemp);	
//	temp+=ctemp;

//	hx=x;

//	ctemp=hx>>8;
//	UART_Put_Char(ctemp);
//	temp+=ctemp;
//	ctemp=hx;
//	UART_Put_Char(ctemp);
//	temp+=ctemp;

//	hy=y;

//	ctemp=hy>>8;
//	UART_Put_Char(ctemp);	
//	temp+=ctemp;
//	ctemp=hy;
//	UART_Put_Char(ctemp);	
//	temp+=ctemp;

//	hz=z;

//	ctemp=hz>>8;
//	UART_Put_Char(ctemp);	 
//	temp+=ctemp;
//	ctemp=hz;
//	UART_Put_Char(ctemp);	 
//	temp+=ctemp;

//	pitch = (int)(angle.pitch*100);
//				   

//	ctemp=pitch>>8;
//	UART_Put_Char(ctemp);	 
//	temp+=ctemp;
//	ctemp=pitch;
//	UART_Put_Char(ctemp);	 
//	temp+=ctemp;


//	roll = (int)(angle.roll*100);

//	ctemp=roll>>8;
//	UART_Put_Char(ctemp);	
//	temp+=ctemp;
//	ctemp=roll;
//	UART_Put_Char(ctemp);	 
//	temp+=ctemp;


// 	yaw = 0;


//	ctemp=yaw>>8;
//	UART_Put_Char(ctemp);	
//	temp+=ctemp;
//	ctemp=yaw;
//	UART_Put_Char(ctemp);	
//	temp+=ctemp;

//	UART_Put_Char(0x00);
//	UART_Put_Char(0x00);
//	UART_Put_Char(0x00);
//	UART_Put_Char(0x00);

//	UART1_Put_Char(temp); 
//}

//void UART3_ReportIMU(void)
//{
//  int16_t ax,ay,az,gx,gy,gz;
//  int16_t hx,hy,hz;
//  int16_t yaw,pitch,roll;
//  uint8_t temp;
//  uint8_t M1_PWM,M2_PWM,M3_PWM,M4_PWM;
//  int16_t BTV;
//  int16_t throt;
//	char ctemp;
//  int32_t presureTEMP;
//	UART_Put_Char(0x88); 
//	UART_Put_Char(0xAF);  
//	UART_Put_Char(0x1C);  
//		 
//	ax=sensor.acc.origin.x;

//	ctemp=ax>>8;
//	UART_Put_Char(ctemp); 
//	temp+=ctemp;
//	ctemp=ax;
//	UART_Put_Char(ctemp); 
//	temp+=sensor.acc.origin.y;

//	ay=sensor.acc.origin.y;

//	ctemp=ay>>8;
//	UART_Put_Char(ctemp); 
//	temp+=ctemp;
//	ctemp=ay;
//	UART_Put_Char(ctemp); 
//	temp+=ctemp;

//	az=sensor.acc.origin.z;

//	ctemp=az>>8;
//	UART_Put_Char(ctemp);
//	temp+=ctemp;
//	ctemp=az;
//	UART_Put_Char(ctemp);
//	temp+=ctemp;

//	gx=sensor.gyro.origin.x;

//	ctemp=gx>>8;
//	UART_Put_Char(ctemp);
//	temp+=ctemp;
//	ctemp=gx;
//	UART_Put_Char(ctemp);
//	temp+=ctemp;

//	gy=sensor.gyro.origin.y;

//	ctemp=gy>>8;
//	UART_Put_Char(ctemp);
//	temp+=ctemp;
//	ctemp=gy;
//	UART_Put_Char(ctemp);
//	temp+=ctemp;

//  gz=sensor.gyro.origin.z;

//	ctemp=gz>>8;
//	UART_Put_Char(ctemp); 
//	temp+=ctemp;
//	ctemp=gz;
//	UART_Put_Char(ctemp);	
//	temp+=ctemp;

//	hx=0;

//	ctemp=hx>>8;
//	UART_Put_Char(ctemp);
//	temp+=ctemp;
//	ctemp=hx;
//	UART_Put_Char(ctemp);
//	temp+=ctemp;

//	hy=0;

//	ctemp=hy>>8;
//	UART_Put_Char(ctemp);	
//	temp+=ctemp;
//	ctemp=hy;
//	UART_Put_Char(ctemp);	
//	temp+=ctemp;

//	hz=0;

//	ctemp=hz>>8;
//	UART_Put_Char(ctemp);	 
//	temp+=ctemp;
//	ctemp=hz;
//	UART_Put_Char(ctemp);	 
//	temp+=ctemp;



//	pitch = (int)(angle.pitch*100);
//				   
//	pitch=0xffff-pitch;
//	ctemp=pitch>>8;
//	UART_Put_Char(ctemp);	 
//	temp+=ctemp;
//	ctemp=pitch;
//	UART_Put_Char(ctemp);	 
//	temp+=ctemp;

//    	roll = (int)(angle.roll*100);
//	 roll=0xffff-roll;
//	ctemp=roll>>8;
//	UART_Put_Char(ctemp);	
//	temp+=ctemp;
//	ctemp=roll;
//	UART_Put_Char(ctemp);	 
//	temp+=ctemp;



//// 	yaw =0;// angle.yaw*10;
// 	yaw = angle.yaw*10;

//	ctemp=yaw>>8;
//	UART_Put_Char(ctemp);	
//	temp+=ctemp;
//	ctemp=yaw;
//	UART_Put_Char(ctemp);	
//	temp+=ctemp;

//	UART_Put_Char(0x00);
//	UART_Put_Char(0x00);
//	UART_Put_Char(0x00);
//	UART_Put_Char(0x00);

//	UART3_Put_Char(temp); 
//	   // 发送数据

// 
//	UART_Put_Char(0x88); 
//	UART_Put_Char(0xAE);  
//	UART_Put_Char(0x12);  

//	throt = Rc_Data.THROTTLE;

//	ctemp=throt>>8;
//	UART_Put_Char(ctemp);	
//	temp+=ctemp;
//	ctemp=throt;
//	UART_Put_Char(ctemp);	 
//	temp+=ctemp;

// 	yaw=Rc_Data.YAW;// angle.yaw*10;


//	ctemp=yaw>>8;
//	UART_Put_Char(ctemp);	
//	temp+=ctemp;
//	ctemp=yaw;
//	UART_Put_Char(ctemp);	
//	temp+=ctemp;
//	
//	roll =Rc_Data.ROLL;

//	ctemp=roll>>8;
//	UART_Put_Char(ctemp);	
//	temp+=ctemp;
//	ctemp=roll;
//	UART_Put_Char(ctemp);	 
//	temp+=ctemp;
//	
//	pitch =Rc_Data.PITCH;

//	ctemp=pitch>>8;
//	UART_Put_Char(ctemp);	 
//	temp+=ctemp;
//	ctemp=pitch;
//	UART_Put_Char(ctemp);	 
//	temp+=ctemp;
//	
//	presureTEMP=Pressure;//AUX1当前高度
//  presureTEMP>>=8;
//	presureTEMP&=0x00FF;
//	ctemp+=presureTEMP;
//	UART_Put_Char(presureTEMP); 
//	presureTEMP=Pressure;
//	presureTEMP&=0x00FF;
//	ctemp+=presureTEMP;
//	UART_Put_Char(presureTEMP); 
//	
//	  presureTEMP=SETPressure;//AUX2设定高度
//	presureTEMP>>=8;
//	ctemp+=presureTEMP;
//	UART_Put_Char(presureTEMP); 
//	
//	presureTEMP=SETPressure;
//	presureTEMP&=0x00FF;
//	ctemp+=presureTEMP;
//	UART_Put_Char(presureTEMP); 
//	
//		  presureTEMP=TEMPSETPressure;//AUX3微调高度
//	presureTEMP>>=8;
//	ctemp+=presureTEMP;
//	UART_Put_Char(presureTEMP); 
//	
//	presureTEMP=TEMPSETPressure;
//	presureTEMP&=0x00FF;
//	ctemp+=presureTEMP;
//	UART_Put_Char(presureTEMP); 
//	
//	presureTEMP=qidongdinggaoTHR;//AUX4设定油门
//	presureTEMP>>=8;
//	ctemp+=presureTEMP;
//	UART_Put_Char(presureTEMP); 
//	
//	presureTEMP=qidongdinggaoTHR;
//	presureTEMP&=0x00FF;
//	ctemp+=presureTEMP;
//	UART_Put_Char(presureTEMP); 
//	
//		presureTEMP=OUT_THROTTLE;//AUX5油门出力
//	presureTEMP>>=8;
//	ctemp+=presureTEMP;
//	UART_Put_Char(presureTEMP); 
//	
//	presureTEMP=OUT_THROTTLE;
//	presureTEMP&=0x00FF;
//	ctemp+=presureTEMP;
//	UART_Put_Char(presureTEMP); 
//	/*
//  presureTEMP=Rc_Data.CH5AUX;//AUX5高度启动
//	presureTEMP>>=8;
//	ctemp+=presureTEMP;
//	UART_Put_Char(presureTEMP);
//	
//	presureTEMP=Rc_Data.CH5AUX;
//	presureTEMP&=0x00FF;
//	ctemp+=presureTEMP;
//	UART_Put_Char(presureTEMP); 
//*/






//    M1_PWM=M1pmw;
//		UART_Put_Char(0);
//	UART_Put_Char(M1_PWM);
//	temp+=M1_PWM;

//	M2_PWM=M2pmw;
//		UART_Put_Char(0);
//	UART_Put_Char(M2_PWM);
//	temp+=M2_PWM;

//	M3_PWM=M3pmw;
//		UART_Put_Char(0);
//	UART_Put_Char(M3_PWM);
//	temp+=M3_PWM;

//	M4_PWM=M4pmw;
//		UART_Put_Char(0);
//	UART_Put_Char(M4_PWM);
//	temp+=M4_PWM;

//   	BTV=BATTVOL;

//	ctemp=BTV>>8;
//	UART_Put_Char(ctemp);	 
//	temp+=ctemp;
//	ctemp=BTV;
//	UART_Put_Char(ctemp);	 
//	temp+=ctemp;
//	UART_Put_Char(temp);
//	
//}

//void NRF_ReportIMU(void)
//{
//  int16_t ax,ay,az,gx,gy,gz;
//  int16_t hx,hy,hz;
//  int16_t yaw,pitch,roll;
//  uint8_t temp;
//	u8 tx_buf[32]={0};	
//	char ctemp;
//	NRF24L01_TX_Mode(); 
//  tx_buf[0]=31; 
//	tx_buf[1]=0x88; 
//	tx_buf[2]=0xAF;  
//	tx_buf[3]=0x1C;  
//	ax=sensor.acc.origin.x;

//	ctemp=ax>>8;
//	tx_buf[4]=ctemp; 
//	temp+=ctemp;
//	ctemp=ax;
//	tx_buf[5]=ctemp; 
//	temp+=sensor.acc.origin.y;

//	ay=sensor.acc.origin.y;

//	ctemp=ay>>8;
//	tx_buf[6]=ctemp; 
//	temp+=ctemp;
//	ctemp=ay;
//	tx_buf[7]=ctemp; 
//	temp+=ctemp;

//	az=sensor.acc.origin.z;

//	ctemp=az>>8;
//	tx_buf[8]=ctemp;
//	temp+=ctemp;
//	ctemp=az;
//	tx_buf[9]=ctemp;
//	temp+=ctemp;

//	gx=sensor.gyro.origin.x;

//	ctemp=gx>>8;
//	tx_buf[10]=ctemp;
//	temp+=ctemp;
//	ctemp=gx;
//	tx_buf[11]=ctemp;
//	temp+=ctemp;

//	gy=sensor.gyro.origin.y;

//	ctemp=gy>>8;
//	tx_buf[12]=ctemp;
//	temp+=ctemp;
//	ctemp=gy;
//	tx_buf[13]=ctemp;
//	temp+=ctemp;

//  gz=sensor.gyro.origin.z;

//	ctemp=gz>>8;
//	tx_buf[14]=ctemp; 
//	temp+=ctemp;
//	ctemp=gz;
//	tx_buf[15]=ctemp;	
//	temp+=ctemp;

//	hx=0;

//	ctemp=hx>>8;
//	tx_buf[16]=ctemp;
//	temp+=ctemp;
//	ctemp=hx;
//	tx_buf[17]=ctemp;
//	temp+=ctemp;

//	hy=0;

//	ctemp=hy>>8;
//	tx_buf[18]=ctemp;	
//	temp+=ctemp;
//	ctemp=hy;
//	tx_buf[19]=ctemp;	
//	temp+=ctemp;

//	hz=0;

//	ctemp=hz>>8;
//	tx_buf[20]=ctemp;	 
//	temp+=ctemp;
//	ctemp=hz;
//	tx_buf[21]=ctemp;	 
//	temp+=ctemp;

//	pitch = (int)(angle.pitch*100);
//				   
//	pitch=0xffff-pitch;
//	ctemp=pitch>>8;
//	tx_buf[22]=ctemp;	 
//	temp+=ctemp;
//	ctemp=pitch;
//	tx_buf[23]=ctemp;	 
//	temp+=ctemp;

//   roll = (int)(angle.roll*100);
//	 roll=0xffff-roll;
//	ctemp=roll>>8;
//	tx_buf[24]=ctemp;	
//	temp+=ctemp;
//	ctemp=roll;
//	tx_buf[25]=ctemp;	 
//	temp+=ctemp;

// 	yaw = angle.yaw*10;

//	ctemp=yaw>>8;
//	tx_buf[26]=ctemp;	
//	temp+=ctemp;
//	ctemp=yaw;
//	tx_buf[27]=ctemp;	
//	temp+=ctemp;

//	tx_buf[28]=0x00;
//	tx_buf[29]=0x00;
//	tx_buf[30]=0x00;
//	tx_buf[31]=0x00;
//NRF24L01_TxPacket(tx_buf);
//////	UART3_Put_Char(temp); 
////	   // 发送数据
//}


