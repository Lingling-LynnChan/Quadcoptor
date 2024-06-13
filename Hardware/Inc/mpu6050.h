#ifndef __MPU6050_H__
#define __MPU6050_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
  float Q0;
  float Q1;
  float Q2;
  float Q3;
} Quaternion;

typedef struct {
  float Pitch;
  float Roll;
  float Yaw;
  Quaternion Quat;
  int16_t MPU_Gyro[3];
  int16_t MPU_Accel[3];
  int32_t MPU_Quat[4];
} GW_Angle;

HAL_StatusTypeDef MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_DMP_Read(GW_Angle* a);
HAL_StatusTypeDef MPU6050_GW_Read(GW_Angle* a, uint32_t dt);
#ifdef __cplusplus
}
#endif

#endif /*__MPU6050_H__ */