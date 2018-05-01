#ifndef __INCLUDE_H
#define	__INCLUDE_H

#define uint8  unsigned char
#define uint16 unsigned int
#include "stm32f10x.h"
#include "gps_config.h"
#include "math.h"
#include "led.h"
#include "app.h"
#include "oled.h"
#include "key.h"
#include "time.h"
#include "I2C.h"
#include "MPU6050.h"
#include "HMC5883.h"
#include "MS5611.h"
#include "IMU.h"
#include "CONTROL.h"
#include "moto.h"
#include "TIM_PWM_IN.h"
#include "ADC.h"
#include "USART.h"
#include "delay.h"
#include "NRF24L01.h"
#include "spi.h"
#include "eeprom.h"
#include "Param.h"
#include "stmflash.h"
#include "send.h"
extern uint16  BATTVOL;
extern uint16 M1pmw,M2pmw,M3pmw,M4pmw;

#endif /* __INCLUDE_H */
