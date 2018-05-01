#ifndef __send_H
#define	__send_H

#include "sys.h"

#include "Param.h"
#include "usart.h"	
void sand_IMU_data(void);
void sand_ACC_GYRO_data(void);
void sand_RC_data(void);
void sand_Motor_data(void);

void sand_PID_shell_data(void);
void sand_PID_core_data(void);

void sand_F1_origin(void);
void sand_F2_origin(void);
void Data_Receive_Prepare(u8 data);
void Data_Receive_Anl(u8 *data_buf,u8 num);


static void Send_Check(u8 head, u8 check_sum);
#endif


