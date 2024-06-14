#include "task.h"
#include <stdint.h>
#include "jlinkp.h"
#include "mpu6050.h"
#include "pid.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "usbd_cdc_if.h"

static void GW_PID_Angle(float dt);

// 软件解算采样滤波时间(单位 ms)
#define GW_DT_MS 3
// 姿态解算和PID计算时间(单位 s)
#define RA_DT_S 0.006f

void GW_Task_1ms(void) {
  HAL_StatusTypeDef hs;
  static uint32_t cnt = 0;
  if (cnt++ >= GW_DT_MS) {
    cnt = 0;
    hs = MPU6050_Sampling();
    if (hs != HAL_OK) {
      jlink("MPU6050_Sampling FAILD\n");
    }
  }
  static uint32_t calc_cnt = 0;
  if (calc_cnt++ >= 6) {
    calc_cnt = 0;
    hs = MPU6050_Read_Angle(RA_DT_S);
    if (hs != HAL_OK) {
      jlink("MPU6050_Read_Angle FAILD\n");
    }
    // jlink("Angle(%f, %f, %f)\n", angle->Pitch, angle->Roll, angle->Yaw);
    // char data[128] = {0};
    // strcat(data, "Angle(");
    // strcat(data, ftoa(GWS_Angle.Pitch));
    // strcat(data, ", ");
    // strcat(data, ftoa(GWS_Angle.Roll));
    // strcat(data, ", ");
    // strcat(data, ftoa(GWS_Angle.Yaw));
    // strcat(data, ")\n");
    // CDC_Transmit_FS((uint8_t*)data, strlen(data));
    GW_PID_Angle(RA_DT_S);
    // TODO 控制电机
  }
}

// 通过姿态数据控制PID
static void GW_PID_Angle(float dt) {
  UNUSED(dt);
}