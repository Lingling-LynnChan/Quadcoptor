#ifndef __MPU6050_H__
#define __MPU6050_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifndef USE_DMP
// MPU6050 IIC地址(AD0引脚控制)
#define MPUC_ADDRESS 0xD0
// 陀螺仪采样率，典型值：0x07(125Hz)
#define MPUC_SMPLRT_DIV 0x19
// 低通滤波频率，典型值：0x06(5Hz)
#define MPUC_CONFIGL 0x1A
// 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define MPUC_GYRO_CONFIG 0x1B
// 加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define MPUC_ACCEL_CONFIG 0x1C
#define MPUC_ACCEL_ADDRESS 0x3B
#define MPUC_ACCEL_XOUT_H 0x3B
#define MPUC_ACCEL_XOUT_L 0x3C
#define MPUC_ACCEL_YOUT_H 0x3D
#define MPUC_ACCEL_YOUT_L 0x3E
#define MPUC_ACCEL_ZOUT_H 0x3F
#define MPUC_ACCEL_ZOUT_L 0x40
#define MPUC_TEMP_OUT_H 0x41
#define MPUC_TEMP_OUT_L 0x42
#define MPUC_GYRO_XOUT_H 0x43
#define MPUC_GYRO_ADDRESS 0x43
#define MPUC_GYRO_XOUT_L 0x44
#define MPUC_GYRO_YOUT_H 0x45
#define MPUC_GYRO_YOUT_L 0x46
#define MPUC_GYRO_ZOUT_H 0x47
#define MPUC_GYRO_ZOUT_L 0x48
// 电源管理，典型值：0x00(正常启用)
#define MPUC_PWR_MGMT_1 0x6B
// IIC地址寄存器(默认数值0x68，只读)
#define MPUC_WHO_AM_I 0x75
#define MPUC_PRODUCT_ID 0x68
#endif

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
#ifndef USE_DMP
  float NormAccelZ;
  int16_t MPU_Gyro_Offset[3];
  int16_t MPU_Accel_Offset[3];
#endif
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