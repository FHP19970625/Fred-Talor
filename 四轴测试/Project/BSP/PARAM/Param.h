#ifndef _PARAM_H_
#define _PARAM_H_
/* Includes ------------------------------------------------------------------*/
#include "include.h"
extern u8 commad;//1=读PID,2=写PID，3=未定义
extern u8 sendms;
extern uint16_t VirtAddVarTab[15];
extern u8 USART_RX_BUF[];
extern u8 USART_TX_BUF[];
void EE_SAVE_ACC_OFFSET(void);
void PID_shell_SAVE(void);
void PID_roll_SAVE(void);
void PID_shell_Read(void);
void PID_roll_Read(void);
extern unsigned char floatTURN2char1000x_char[];
void floatTURN2char1000x(float data);//浮点数转为1字节数
void paramLoad(void);
void USART2_IRQHandler(void);                	//串口2中断服务程序
void Receive_NRF_Data(void);
#endif /* __EEPROM_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

