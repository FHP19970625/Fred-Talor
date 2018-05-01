#ifndef _PARAM_H_
#define _PARAM_H_
/* Includes ------------------------------------------------------------------*/
#include "include.h"
extern u8 commad;//1=��PID,2=дPID��3=δ����
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
void floatTURN2char1000x(float data);//������תΪ1�ֽ���
void paramLoad(void);
void USART2_IRQHandler(void);                	//����2�жϷ������
void Receive_NRF_Data(void);
#endif /* __EEPROM_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

