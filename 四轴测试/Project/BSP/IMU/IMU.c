/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：IMU.c
 * 描述    ：姿态解算         
 * 实验平台：HT飞控
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team 
**********************************************************************************/
#include "IMU.h"
#include "math.h"
#include "MPU6050.h"
struct _angle angle;
#define a   0.01                // ????a(0-1) 
float valuex;                    //????? 
float valuey;
float valuez;
char value;
float new_value;                 //  ?????  
#define A 10
//char value;
int Filter_Value;
int Value;
#define N  12
int16_t ggx;
int16_t ggy;
int16_t ggz;
/*	
	Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	R:测量噪声，R增大，动态响应变慢，收敛稳定性变好	
*/
#define KALMAN_Q        0.02
#define KALMAN_R        64.0000
/*           卡尔曼对三个轴加速度进行滤波处理           */
static double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
 }
static double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
 }
static double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
 }

 
//   快速求平方根倒数
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration （第一次牛顿迭代）
	return y;
} 
//  求float型数据绝对值
float FL_ABS(float x)
{
   if(x < 0)  return -x;
	 else return x; 
}
/*   采用三角函数的泰勒展开式 求近似值*/
float COS(float x)
{
	float result;
  result = 1 - x * x/2;
	return result; 
}

float SIN(float y)
{
	float result;
  result = y - y * y * y /6;
	return result; 
}
/***********************************************
  * @brief  可变增益自适应参数
  * @param  None
  * @retval None
************************************************/
float VariableParameter(float error)
{
	float  result = 0;
	
	if(error < 0)
	{
	   error = -error;
	}
  if(error >0.8f)
	{
	   error = 0.8f;
	}
	result = 1 - 1.28 * error;
	if(result < 0)
	{
	   result = 0;
	}
	return result;
}

float filtergx()   
{   
float new_value;   
new_value = sensor.gyro.origin.x;   
return 0.5*valuex + (1-0.5)*new_value;  
 }  
float filtergy()   
{   
float new_value;   
new_value = sensor.gyro.origin.y;   
return 0.5*valuey + (1-0.5)*new_value;  
 }
float filtergz()   
{   
float new_value;   
new_value = sensor.gyro.origin.z;   
return 0.5*valuez + (1-0.5)*new_value;  
 }
// 限幅滤波
char filterd()
{
   char  new_value;
   new_value = sensor.acc.origin.x;
   if ( ( new_value - value > A ) || ( value - new_value > A ))
      return value;
   return new_value;
         
}

//中位值滤波法
char filterz()
{
   char value_buf[N];
   char count,i,j,temp;
   for ( count=0;count<N;count++)
   {
      value_buf[count] = sensor.acc.origin.x;
      delay(1);
   }
   for (j=0;j<N-1;j++)
   {
      for (i=0;i<N-j;i++)
      {
         if ( value_buf[i]>value_buf[i+1] )
         {
            temp = value_buf[i];
            value_buf[i] = value_buf[i+1]; 
             value_buf[i+1] = temp;
         }
      }
   }
   return value_buf[(N-1)/2];
}
char filteru()
{
   char count=0;
   char new_value;
   new_value = sensor.acc.origin.x;
   while (value !=new_value)
   {
      count++;
      if (count>=N)   return new_value;
       delay(1);
      new_value = sensor.acc.origin.x;
   }
   return value;    
}

/*************************************/
 /**************************************************************************
函数功能：一阶互补滤波
入口参数：加速度、角速度
返回  值：无
**************************************************************************/

float K1 =0.05; // ??????????
float dt=0.002;//取样时间
float angle1;
float PI=3.1415926f;
u16 angle_new;
float Yijielvbo(float angle_x, float angle_z)//?????????????
{	
	  u16 angleAx;
		float gyro_m;
		gyro_m=(sensor.gyro.origin.y - sensor.gyro.quiet.y )* Gyro_G;
		angleAx=atan2(angle_x,angle_z)*180/PI;
	
    angle1 = K1 * angleAx+ (1-K1) * (angle1 + gyro_m * dt);
		return angle1;
}

#define  IIR_ORDER     4      //使用IIR滤波器的阶数
double b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //系数b
double a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//系数a
double InPut_IIR[3][IIR_ORDER+1] = {0};
double OutPut_IIR[3][IIR_ORDER+1] = {0};


void Prepare_Data(void)
{
	MPU6050_Dataanl();          //读取6050
	Multiple_Read_HMC5883L();   //读取地磁数据
//sensor.acc.averag.x=filteru();
	valuex=filtergx();
	valuey=filtergy();
	valuez=filtergz();
//	ggx=filtergx();
//	ggy=filtergy();
//	ggz=filtergz();
	sensor.acc.averag.x = KalmanFilter_x(sensor.acc.origin.x,KALMAN_Q,KALMAN_R);  // ACC X轴卡尔曼滤波
	sensor.acc.averag.y = KalmanFilter_y(sensor.acc.origin.y,KALMAN_Q,KALMAN_R);  // ACC Y轴卡尔曼滤波
	sensor.acc.averag.z = KalmanFilter_z(sensor.acc.origin.z,KALMAN_Q,KALMAN_R);  // ACC Z轴卡尔曼滤波
//	sensor.acc.averag.x = IIR_I_Filter(sensor.acc.origin.x, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
//	sensor.acc.averag.y = IIR_I_Filter(sensor.acc.origin.y, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
//	sensor.acc.averag.z = IIR_I_Filter(sensor.acc.origin.z, InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
}

//float qa0, qa1, qa2, qa3;
//#define Kp 0.8f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
//#define Ki 0.0015f                     // integral gain governs rate of convergence of gyroscope biases
//#define halfT 0.00125f                 // 采样周期的一半  本程序 2.5MS 采集一次  所以 halfT是1.25MS

/**************************************
 * 函数名：Get_Attitude
 * 描述  ：得到当前姿态
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 *************************************/
void Get_Attitude(void)
{
	IMUupdate(sensor.gyro.radian.x,
						sensor.gyro.radian.y,
						sensor.gyro.radian.z,
						sensor.acc.averag.x,
	          sensor.acc.averag.y,
	          sensor.acc.averag.z,
	X_HMC,
	Y_HMC,
	Z_HMC
	);	
}
#define Kp 1.0f  //10                     //proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.002f  //0.002                     //integral gain governs rate of convergence of gyroscope biases
#define halfT 0.00125f  //0.001                  // half the sample period采样周期的一半

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az ,float mx, float my, float mz)
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;

  // 先把这些用得到的值算好
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
  norm = Q_rsqrt(ax*ax + ay*ay + az*az);       
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
  //把加计的三维向量转成单位向量。

  norm = Q_rsqrt(mx*mx + my*my + mz*mz);          
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  //从机体坐标系的电子罗盘测到的矢量转成地理坐标系下的磁场矢量hxyz（测量值）
  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);
  
//  计算地理坐标系下的磁场矢量bxyz（参考值）。
//  因为地理地磁水平夹角，我们已知是0度（抛去磁偏角的因素，固定向北），所以by=0，bx=某值
//  但地理参考地磁矢量在垂直面上也有分量bz，地球上每个地方都是不一样的。
//  我们无法得知，也就无法用来融合（有更适合做垂直方向修正融合的加速度计），所以直接从测量值hz上复制过来，bz=hz。
//  磁场水平分量，参考值和测量值的大小应该是一致的(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))。
//  因为by=0，所以就简化成(bx*bx)  = ((hx*hx) + (hy*hy))。可算出bx。
           
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;     
  
 
//  这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
//  根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
//  所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。
  
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
 
 // 我们把地理坐标系上的磁场矢量bxyz，转到机体上来wxyz。
 // 因为by=0，所以所有涉及到by的部分都被省略了。
 // 类似上面重力vxyz的推算，因为重力g的gz=1，gx=gy=0，所以上面涉及到gxgy的部分也被省略了
  
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
  
  //现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

  
 // axyz是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
//  axyz是测量得到的重力向量，vxyz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
// 那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
//  向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，exyz就是两个重力向量的叉积。
//  这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。
  
if(ex != 0.0f && ey != 0.0f && ez != 0.0f){
  exInt = exInt + ex * Ki * halfT;
  eyInt = eyInt + ey * Ki * halfT;	
  ezInt = ezInt + ez * Ki * halfT;

  // 用叉积误差来做PI修正陀螺零偏
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;

  }

  // 四元数微分方程
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  
  // 四元数规范化
  norm = Q_rsqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;

  angle.yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* RtA; // yaw
  angle.pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* RtA; // pitch
  angle.roll = -atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* RtA; // roll
} 












