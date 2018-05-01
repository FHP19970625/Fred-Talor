#ifndef __MS5611_H
#define	__MS5611_H
#include "stm32f10x.h"
/* MPU6050 Register Address ------------------------------------------------------------*/
#define MS561101BA_ADC_RD          0x00
#define	MS561101BA_PROM_RD 	       0xA0
#define MS561101BA_PROM_CRC        0xAE

#define MS561101BA_SlaveAddress    0xEE  //MS5611的地址
#define MS561101BA_RST             0x1E  //cmd 复位

#define	MS561101BA_D2_OSR_4096   0x58	// 9.04 mSec conversion time ( 110.62 Hz)
#define	MS561101BA_D1_OSR_4096   0x48

#define MS5611_OSR256					 		 0x40
#define MS5611_OSR512					 		 0x42
#define MS5611_OSR1024					   0x44
#define MS5611_OSR2048					   0x46
#define MS5611_OSR4096					   0x48
#define FILTER_num 20
extern uint32_t Pressure,Pressure_old,qqp;				//大气压
extern uint32_t SETPressure;//启动气压定高时气压值
extern u8 BARO_MODE;//0=没开气压定高，1=开启气压定高
extern uint32_t qidongdinggaoTHR;//启动气压定高时油门值
extern uint32_t TEMPSETPressure;//临时小范围高度值
extern uint32_t OUT_THROTTLE;//启动气压定高时输出油门值
u8  MS5611_init(void);
float Get_High(void);

#endif
