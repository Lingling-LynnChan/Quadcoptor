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
  float X;
  float Y;
  float Z;
} Vector3f;

typedef struct {
  float Pitch;
  float Roll;
  float Yaw;
  Quaternion Quat;
  int16_t MPU_Gyro[3];
  int16_t MPU_Accel[3];
  int32_t MPU_Quat[4];
  float NormAccelZ;
  int16_t MPU_Gyro_Offset[3];
  int16_t MPU_Accel_Offset[3];
} GW_Angle;

extern GW_Angle GWS_Angle;

HAL_StatusTypeDef MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_Sampling(void);
HAL_StatusTypeDef MPU6050_Read_Angle(float dt);
HAL_StatusTypeDef MPU6050_Check(void);

#ifdef __cplusplus
}
#endif

#endif /*__MPU6050_H__ */