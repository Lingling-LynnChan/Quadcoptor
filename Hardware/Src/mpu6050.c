#include "mpu6050.h"
#include <math.h>
#include "i2c.h"
#include "kalman.h"

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

GW_Angle GWS_Angle;

HAL_StatusTypeDef MPU6050_Init(void) {
  HAL_StatusTypeDef hs;
  // 复位
  HAL_Delay(100);
  hs = GW_I2C1_Send_Byte(MPUC_ADDRESS, MPUC_PWR_MGMT_1, 0x80);
  if (hs != HAL_OK) {
    jlink("MPU6050 RESET FAILD\n");
    return hs;
  }
  HAL_Delay(100);
  // 陀螺仪采样率
  hs = GW_I2C1_Send_Byte(MPUC_ADDRESS, MPUC_SMPLRT_DIV, 0x02);
  if (hs != HAL_OK) {
    jlink("MPU6050 SMPLRT_DIV FAILD\n");
    return hs;
  }
  // 设置设备时钟源，陀螺仪Z轴
  hs = GW_I2C1_Send_Byte(MPUC_ADDRESS, MPUC_PWR_MGMT_1, 0x03);
  if (hs != HAL_OK) {
    jlink("MPU6050 PWR_MGMT_1 FAILD\n");
    return hs;
  }
  // 低通滤波频率: 42Hz
  hs = GW_I2C1_Send_Byte(MPUC_ADDRESS, MPUC_CONFIGL, 0x03);
  if (hs != HAL_OK) {
    jlink("MPU6050 CONFIGL FAILD\n");
    return hs;
  }
  // 陀螺仪自检及量程: 2000dps
  hs = GW_I2C1_Send_Byte(MPUC_ADDRESS, MPUC_GYRO_CONFIG, 0x18);
  if (hs != HAL_OK) {
    jlink("MPU6050 GYRO_CONFIG FAILD\n");
    return hs;
  }
  // 加速计自检及量程: 4G
  hs = GW_I2C1_Send_Byte(MPUC_ADDRESS, MPUC_ACCEL_CONFIG, 0x09);
  if (hs != HAL_OK) {
    jlink("MPU6050 ACCEL_CONFIG FAILD\n");
    return hs;
  }
  uint8_t mpuid;
  hs = GW_I2C1_Read_Byte(MPUC_ADDRESS, MPUC_WHO_AM_I, &mpuid);
  if (hs != HAL_OK || mpuid != MPUC_PRODUCT_ID) {
    jlink("MPU6050 WHO_AM_I FAILD\n");
    return HAL_ERROR;
  }
  // 偏移标定
  hs = MPU6050_Check();
  if (hs != HAL_OK) {
    jlink("MPU6050 CHECK FAILD\n");
    return hs;
  }
  return HAL_OK;
}
// 采样
HAL_StatusTypeDef MPU6050_Sampling(void) {
  HAL_StatusTypeDef hs;
  uint8_t accel_buf[6], gyro_buf[6];
  // 读取加速度计
  hs = GW_I2C1_Read_Data(MPUC_ADDRESS, MPUC_ACCEL_ADDRESS, accel_buf, 6);
  if (hs != HAL_OK) {
    return hs;
  }
  // 读取陀螺仪
  hs = GW_I2C1_Read_Data(MPUC_ADDRESS, MPUC_GYRO_ADDRESS, gyro_buf, 6);
  if (hs != HAL_OK) {
    return hs;
  }
  // 滤波处理
  for (uint8_t i = 0; i < 3; i++) {
    // 加速度计数据转换
    GWS_Angle.MPU_Accel[i] =
        (((int16_t)accel_buf[i << 1] << 8) | accel_buf[(i << 1) + 1]) -
        GWS_Angle.MPU_Accel_Offset[i];
    // 加速度做卡尔曼滤波
    static GW_Kalman_Filter kalman[3] = {{0.02, 0, 0, 0, 0.001, 0.543},
                                         {0.02, 0, 0, 0, 0.001, 0.543},
                                         {0.02, 0, 0, 0, 0.001, 0.543}};
    GW_Kalman_Filter_V1(&kalman[i], (float)GWS_Angle.MPU_Accel[i]);
    GWS_Angle.MPU_Accel[i] = (int16_t)kalman[i].Out;
  }
  for (uint8_t i = 0; i < 3; i++) {
    // 陀螺仪数据转换
    GWS_Angle.MPU_Gyro[i] =
        (((int16_t)gyro_buf[i << 1] << 8) | gyro_buf[(i << 1) + 1]) -
        GWS_Angle.MPU_Gyro_Offset[i];
    // 角速度做一阶低通滤波
    const float factor = 0.15f;  // 滤波因素
    static float last[3] = {0, 0, 0};
    last[i] = last[i] * (1 - factor) + GWS_Angle.MPU_Gyro[i] * factor;
    GWS_Angle.MPU_Gyro[i] = (int16_t)last[i];
  }
  return HAL_OK;
}
// WTF算法开方
static float rsqrt(float number) {
  long i;
  float x2, y;
  const float threehalfs = 1.5F;
  x2 = number * 0.5F;
  y = number;
  i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (threehalfs - (x2 * y * y));
  return y;
}
// 常量
const float RtA = 57.2957795f;                  // 弧度转角度
const float AtR = 1 / RtA;                      // 角度转弧度
const float Gyro_G = 1.f / (65536.f / 4000.f);  // 陀螺仪读数转角度
const float Gyro_Gr = Gyro_G * AtR;             // 陀螺仪得到的度每秒
// 姿态解算
HAL_StatusTypeDef MPU6050_Read_Angle(float dt) {
  static Vector3f GyroInteg = {0.f, 0.f, 0.f};
  static float Kp = 0.8f;
  static float Ki = 0.0003f;
  static Quaternion NumQ = {1, 0, 0, 0};
  float ht = dt * 0.5f;
  Vector3f Gravity, Accel, Gyro, AccelGravity;
  float NormQuat;
  // 提取重力分量
  Gravity.X = 2 * (NumQ.Q1 * NumQ.Q3 - NumQ.Q0 * NumQ.Q2);
  Gravity.Y = 2 * (NumQ.Q0 * NumQ.Q1 + NumQ.Q2 * NumQ.Q3);
  Gravity.Z = 1 - 2 * (NumQ.Q1 * NumQ.Q1 + NumQ.Q2 * NumQ.Q2);
  // 加速度归一化
  NormQuat = rsqrt(GWS_Angle.MPU_Accel[0] * GWS_Angle.MPU_Accel[0] +
                   GWS_Angle.MPU_Accel[1] * GWS_Angle.MPU_Accel[1] +
                   GWS_Angle.MPU_Accel[2] * GWS_Angle.MPU_Accel[2]);
  Accel.X = GWS_Angle.MPU_Accel[0] * NormQuat;
  Accel.Y = GWS_Angle.MPU_Accel[1] * NormQuat;
  Accel.Z = GWS_Angle.MPU_Accel[2] * NormQuat;
  // 计算向量积得到重力加速度
  AccelGravity.X = Accel.Y * Gravity.Z - Accel.Z * Gravity.Y;
  AccelGravity.Y = Accel.Z * Gravity.X - Accel.X * Gravity.Z;
  AccelGravity.Z = Accel.X * Gravity.Y - Accel.Y * Gravity.X;
  // 加速度做积分得到角速度的补偿值
  GyroInteg.X += AccelGravity.X * Ki;
  GyroInteg.Y += AccelGravity.Y * Ki;
  GyroInteg.Z += AccelGravity.Z * Ki;
  // 角速度融合加速度积分补偿值
  Gyro.X = GWS_Angle.MPU_Gyro[0] * Gyro_Gr + Kp * AccelGravity.X + GyroInteg.X;
  Gyro.Y = GWS_Angle.MPU_Gyro[1] * Gyro_Gr + Kp * AccelGravity.Y + GyroInteg.Y;
  Gyro.Z = GWS_Angle.MPU_Gyro[2] * Gyro_Gr + Kp * AccelGravity.Z + GyroInteg.Z;
  // 解四元数微分方程并进行积分
  NumQ.Q0 -= ht * (NumQ.Q1 * Gyro.X + NumQ.Q2 * Gyro.Y + NumQ.Q3 * Gyro.Z);
  NumQ.Q1 += ht * (NumQ.Q0 * Gyro.X - NumQ.Q3 * Gyro.Y + NumQ.Q2 * Gyro.Z);
  NumQ.Q2 += ht * (NumQ.Q3 * Gyro.X + NumQ.Q0 * Gyro.Y - NumQ.Q1 * Gyro.Z);
  NumQ.Q3 -= ht * (NumQ.Q2 * Gyro.X - NumQ.Q1 * Gyro.Y + NumQ.Q0 * Gyro.Z);
  // 四元数归一化
  NormQuat = rsqrt(NumQ.Q0 * NumQ.Q0 + NumQ.Q1 * NumQ.Q1 + NumQ.Q2 * NumQ.Q2 +
                   NumQ.Q3 * NumQ.Q3);
  NumQ.Q0 *= NormQuat;
  NumQ.Q1 *= NormQuat;
  NumQ.Q2 *= NormQuat;
  NumQ.Q3 *= NormQuat;
  // 通过四元数计算欧拉角
  // Z方向向量
  float zToX = 2 * NumQ.Q1 * NumQ.Q3 - 2 * NumQ.Q0 * NumQ.Q2;
  float zToY = 2 * NumQ.Q2 * NumQ.Q3 + 2 * NumQ.Q0 * NumQ.Q1;
  float zToZ = 1 - 2 * NumQ.Q1 * NumQ.Q1 - 2 * NumQ.Q2 * NumQ.Q2;
  // 俯仰角
  GWS_Angle.Pitch = asinf(zToX) * RtA;
  // 横滚角
  GWS_Angle.Roll = atan2f(zToY, zToZ) * RtA;
  // 偏航角: 四元数计算
  GWS_Angle.Yaw = atan2f(2 * NumQ.Q1 * NumQ.Q2 + 2 * NumQ.Q0 * NumQ.Q3,
                         1 - 2 * NumQ.Q2 * NumQ.Q2 - 2 * NumQ.Q3 * NumQ.Q3) *
                  RtA;
  /* 偏航角: 陀螺仪积分
  float yaw_G = GWS_Angle.MPU_Gyro[3] * Gyro_G;  // z轴角速度转rad/s
  if ((yaw_G > 0.8f) || (yaw_G < -0.8f)) {       // 太小不看
    GWS_Angle.Yaw += yaw_G * dt;                 // 角速度积分成偏航角
  }*/
  // Z方向的垂直加速度
  GWS_Angle.NormAccelZ = GWS_Angle.MPU_Accel[0] * zToX +
                         GWS_Angle.MPU_Accel[1] * zToY +
                         GWS_Angle.MPU_Accel[2] * zToZ;
  // 加速度做卡尔曼滤波
  static GW_Kalman_Filter zFilter = {0.02, 0, 0, 0, 0.001, 0.0};
  GW_Kalman_Filter_V1(&zFilter, GWS_Angle.NormAccelZ);
  GWS_Angle.NormAccelZ = zFilter.Out;
  return HAL_OK;
}
// 复位
HAL_StatusTypeDef MPU6050_Reset(void) {
  if (GW_I2C1_Send_Byte(MPUC_ADDRESS, MPUC_PWR_MGMT_1, 0x80) != HAL_OK) {
    return HAL_ERROR;
  }
  HAL_Delay(100);
  return HAL_OK;
}
// 校准
HAL_StatusTypeDef MPU6050_Check(void) {
  HAL_StatusTypeDef hs;
  // 陀螺仪静止判定: 抖动范围
  const int8_t MAX_GYRO_QUIET = 5;
  const int8_t MIN_GYRO_QUIET = -5;
  // 陀螺仪静止判定: 记录两次数据和差值
  int16_t LastGyro[3] = {0};
  int16_t ErrorGyro[3];
  // 朴素初始化
  GWS_Angle.MPU_Accel_Offset[0] = 0;
  GWS_Angle.MPU_Accel_Offset[1] = 0;
  GWS_Angle.MPU_Accel_Offset[2] = 8192;  // 重力加速度1G
  GWS_Angle.MPU_Gyro_Offset[0] = 0;
  GWS_Angle.MPU_Gyro_Offset[1] = 0;
  GWS_Angle.MPU_Gyro_Offset[2] = 0;
  // 静止判定
  uint8_t k = 30;
  while (k--) {
    do {
      HAL_Delay(10);
      hs = MPU6050_Sampling();
      if (hs != HAL_OK) {
        return hs;
      }
      for (int16_t i = 0; i < 3; i++) {
        ErrorGyro[i] = GWS_Angle.MPU_Gyro[i] - LastGyro[i];
        LastGyro[i] = GWS_Angle.MPU_Gyro[i];
      }
    } while (
        (ErrorGyro[0] > MAX_GYRO_QUIET) ||
        (ErrorGyro[0] < MIN_GYRO_QUIET)  // 标定静止
        || (ErrorGyro[1] > MAX_GYRO_QUIET) || (ErrorGyro[1] < MIN_GYRO_QUIET) ||
        (ErrorGyro[2] > MAX_GYRO_QUIET) || (ErrorGyro[2] < MIN_GYRO_QUIET));
  }
  int32_t accel_buf[3] = {0};
  int32_t gyro_buf[3] = {0};
  for (int16_t i = 0; i < 356; i++) {  // 水平校准
    HAL_Delay(10);
    hs = MPU6050_Sampling();
    if (hs != HAL_OK) {
      return hs;
    }
    if (100 <= i) {  // 抛弃前100组数据
      for (uint8_t k = 0; k < 3; k++) {
        accel_buf[k] += GWS_Angle.MPU_Accel[k];
      }
      for (uint8_t k = 0; k < 3; k++) {
        gyro_buf[k] += GWS_Angle.MPU_Gyro[k];
      }
    }
  }
  for (int8_t i = 0; i < 3; i++) {
    GWS_Angle.MPU_Accel_Offset[i] = accel_buf[i] >> 8;
  }
  for (int8_t i = 0; i < 3; i++) {
    GWS_Angle.MPU_Gyro_Offset[i] = gyro_buf[i] >> 8;
  }
  int16_t offsets[6] = {
      GWS_Angle.MPU_Accel_Offset[0], GWS_Angle.MPU_Accel_Offset[1],
      GWS_Angle.MPU_Accel_Offset[2], GWS_Angle.MPU_Gyro_Offset[0],
      GWS_Angle.MPU_Gyro_Offset[1],  GWS_Angle.MPU_Gyro_Offset[2]};
  // TODO FLASH_write(offsets, 6);
  UNUSED(offsets);
  return HAL_OK;
}
