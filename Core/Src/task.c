#include "task.h"
#include <stdint.h>
#include "mpu6050.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "usbd_cdc_if.h"

#ifdef USE_DMP
// 硬件解算时间(单位 ms)
#define GW_DT 6
#else
// 软件解算时间(单位 ms)
#define GW_DT 3
#endif
// 角度积分时间(单位 s)
#define RA_DT 0.006f

void GW_Task_1ms(void) {
  HAL_StatusTypeDef hs;
  GW_Angle* angle = &GWS_Angle;
  static uint32_t cnt = 0;
  if (cnt++ >= GW_DT) {
    cnt = 0;
    hs = MPU6050_Sampling();
    if (hs != HAL_OK) {
      jlink("MPU6050_Sampling FAILD\n");
    }
  }
  static uint32_t calc_cnt = 0;
  if (calc_cnt++ >= 6) {
    calc_cnt = 0;
    hs = MPU6050_Read_Angle(RA_DT);
    if (hs != HAL_OK) {
      jlink("MPU6050_Read_Angle FAILD\n");
    }
    // TODO 计算PID，控制电机
    jlink("Angle(%f, %f, %f)\n", angle->Pitch, angle->Roll, angle->Yaw);
    // CDC_Transmit_FS(buff, 12);
  }
}
