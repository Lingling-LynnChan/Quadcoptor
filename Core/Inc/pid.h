#ifndef __GW_PID_H__
#define __GW_PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "mpu6050.h"

typedef volatile struct {
  float Desired;         // 期望值
  float Measured;        // 实际值
  float Offset;          // 偏差值
  float PrevError;       // 前一次误差
  float Kp;              // 比例参数
  float Ki;              // 积分参数
  float Kd;              // 微分参数
  float Integ;           // 积分值
  float IntegLimitHigh;  // 积分上限
  float IntegLimitLow;   // 积分下限
  float Out;             // 输出值
  float OutLimitHigh;    // 输出上限
  float OutLimitLow;     // 输出下限
} GW_PID;

extern GW_PID GWS_PID;

void GW_PID_Reset();
void GW_PID_Update(float dt);

#ifdef __cplusplus
}
#endif

#endif /*__GW_PID_H__ */