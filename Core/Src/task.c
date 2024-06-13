#include "task.h"
#include <stdint.h>
#include "mpu6050.h"
#include "stm32f1xx_hal.h"

#define CLK_DEF(xx) static uint16_t c##xx##ms = 0
#define IF_CLK_MS(xx) if (c##xx##ms++ >= xx ? (c##xx##ms = 0, 1) : 0)
#define ELIF_CLK_MS(xx) else if (c##xx##ms++ >= xx ? (c##xx##ms = 0, 1) : 0)

void GW_Task_1ms(void) {
  static GW_Angle angle;
  CLK_DEF(2);
  CLK_DEF(5);
  HAL_StatusTypeDef hs;
  float pitch, roll, yaw;
  int timer = 0;
  IF_CLK_MS(2) {  // 2ms 一次的软件解算
    hs = MPU6050_GW_Read(&angle, 2);
    if (hs != HAL_OK) {
      jlink("MPU6050_DMP_Read failed\n");
    }
    timer = 2;
  }
  IF_CLK_MS(5) {  // 5ms 一次的硬件解算
    hs = MPU6050_DMP_Read(&angle);
    if (hs != HAL_OK) {
      jlink("MPU6050_DMP_Read failed\n");
    }
    timer = 5;
  }
  jlink("MPU6050: %f, %f, %f By Timer %d\n", pitch, roll, yaw, timer);
}
