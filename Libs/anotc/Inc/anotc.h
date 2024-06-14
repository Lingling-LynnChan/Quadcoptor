#ifndef __ANOTC_H__
#define __ANOTC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
  ANOTC_VER = 0x00,        // 版本号
  ANOTC_STATUS = 0x01,     // 状态
  ANOTC_MPU_MAGIC = 0X02,  // MPU校准
  ANOTC_RCDATA = 0x03,     // 遥控器数据
  ANOTC_GPSDATA = 0x04,    // GPS数据
  ANOTC_POWER = 0x05,      // 电源数据
  ANOTC_MOTOR = 0x06,      // 电机数据
  ANOTC_SENSER2 = 0x07,    // 传感器数据
  ANOTC_RESERD1 = 0x08,    // 保留1
  ANOTC_RESERD2 = 0x09,    // 保留2
  ANOTC_FLY_MODE = 0x0A,   // 飞行模式
  ANOTC_PID1 = 0x10,       // PID1:角速度
  ANOTC_PID2 = 0x11,       // PID2:角度
  ANOTC_PID3 = 0x12,       // PID3:高度
  ANOTC_PID4 = 0x13,       // PID4
  ANOTC_PID5 = 0x14,       // PID5
  ANOTC_PID6 = 0x15,       // PID6
  ANOTC_CHECK = 0xEF       // 校验
} ANOTC_SEND;

extern int16_t GWS_PID_Check;
void ANOTC_Recive(int8_t* pt);     
void ANOTC_Polling(void);        
void ANOTC_Send(ANOTC_SEND type); 

#ifdef __cplusplus
}
#endif

#endif /*__ANOTC_H__ */