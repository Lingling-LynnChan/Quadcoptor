#include "pid.h"

GW_PID GWS_PID;

// 复位PID
void GW_PID_Reset() {
  GWS_PID.Measured = 0;
  GWS_PID.Desired = 0;
  GWS_PID.Integ = 0;
  GWS_PID.PrevError = 0;
  GWS_PID.Out = 0;
}

#define LMT(v, l, h) ((v) < (l) ? (l) : (v) > (h) ? (h) : (v))
// 更新PID
void GW_PID_Update(float dt) {
  // 计算误差+偏差前馈
  float error = GWS_PID.Desired - GWS_PID.Measured + GWS_PID.Offset;
  // 积分累加
  GWS_PID.Integ += error * dt;
  // 积分限幅
  // GWS_PID.Integ = LMT(GWS_PID.Integ, GWS_PID.IntegLimitLow,
  // GWS_PID.IntegLimitHigh); 误差求微分
  float deriv = (error - GWS_PID.PrevError) / dt;
  // PID输出
  GWS_PID.Out =
      GWS_PID.Kp * error + GWS_PID.Ki * GWS_PID.Integ + GWS_PID.Kd * deriv;
  // 输出限幅
  // GWS_PID.Out = LMT(GWS_PID.Out, GWS_PID.OutLimitLow, GWS_PID.OutLimitHigh);
  // 保存误差
  GWS_PID.PrevError = error;
}