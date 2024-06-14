#include "mpu6050.h"
#include <math.h>

GW_Angle GWS_Angle;

#ifdef USE_DMP

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
static int8_t GW_Inv_Matrix[9] = {
    1, 0, 0,  //
    0, 1, 0,  //
    0, 0, 1,  //
};
static unsigned short inv_row_2_scale(const signed char* row) {
  unsigned short b;
  if (row[0] > 0)
    b = 0;
  else if (row[0] < 0)
    b = 4;
  else if (row[1] > 0)
    b = 1;
  else if (row[1] < 0)
    b = 5;
  else if (row[2] > 0)
    b = 2;
  else if (row[2] < 0)
    b = 6;
  else
    b = 7;  // error
  return b;
}
static unsigned short inv_orientation_matrix_to_scalar(const signed char* mtx) {
  unsigned short scalar;
  /*
     XYZ  010_001_000 Identity Matrix
     XZY  001_010_000
     YXZ  010_000_001
     YZX  000_010_001
     ZXY  001_000_010
     ZYX  000_001_010
   */
  scalar = inv_row_2_scale(mtx);
  scalar |= inv_row_2_scale(mtx + 3) << 3;
  scalar |= inv_row_2_scale(mtx + 6) << 6;
  return scalar;
}

static int run_test(void) {
  int32_t Gyro[3], Accel[3];
  int result = mpu_run_self_test(Gyro, Accel);
  if (!(result & 0x3)) {
    return -1;
  }
  float Gyro_Sens;
  mpu_get_gyro_sens(&Gyro_Sens);
  Gyro[0] = (int32_t)(Gyro[0] * Gyro_Sens);
  Gyro[1] = (int32_t)(Gyro[1] * Gyro_Sens);
  Gyro[2] = (int32_t)(Gyro[2] * Gyro_Sens);
  dmp_set_gyro_bias(Gyro);
  uint16_t Accel_Sens;
  mpu_get_accel_sens(&Accel_Sens);
  Accel[0] *= Accel_Sens;
  Accel[1] *= Accel_Sens;
  Accel[2] *= Accel_Sens;
  dmp_set_accel_bias(Accel);
  return 0;
}

HAL_StatusTypeDef MPU6050_Init(void) {
  struct int_param_s int_param;
  int err;
  err = mpu_init(&int_param);
  if (err) {
    return HAL_ERROR;
  }
  // 使能陀螺仪和加速度计
  err = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  if (err) {
    return HAL_ERROR;
  }
  // 设置FIFO
  err = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  if (err) {
    return HAL_ERROR;
  }
  // 设置陀螺仪量程
  err = mpu_set_gyro_fsr(2000);  // 2000dps
  if (err) {
    return HAL_ERROR;
  }
  // 设置加速度计量程
  err = mpu_set_accel_fsr(4);  // 4g
  if (err) {
    return HAL_ERROR;
  }
  // 设置采样率
  err = mpu_set_sample_rate(500);  // 500Hz: 2ms
  if (err) {
    return HAL_ERROR;
  }
  // 加载DMP
  err = dmp_load_motion_driver_firmware();
  if (err) {
    return HAL_ERROR;
  }
  // 设置陀螺仪方向
  err = dmp_set_orientation(inv_orientation_matrix_to_scalar(GW_Inv_Matrix));
  if (err) {
    return HAL_ERROR;
  }
  // 设置FIFO速率
  err = dmp_set_fifo_rate(200);  // 200Hz: 5ms
  if (err) {
    return HAL_ERROR;
  }
  // 使能DMP功能
  err = dmp_enable_feature(         //
      DMP_FEATURE_6X_LP_QUAT |      // 6轴低功耗四元数
      DMP_FEATURE_TAP |             // 敲击检测
      DMP_FEATURE_ANDROID_ORIENT |  // 朝向数据使用安卓标准
      DMP_FEATURE_SEND_RAW_ACCEL |  // 发送原始加速度计数据
      DMP_FEATURE_SEND_CAL_GYRO |   // 发送校准后的陀螺仪数据
      DMP_FEATURE_GYRO_CAL          // 陀螺仪校准
  );
  if (err) {
    return HAL_ERROR;
  }
  // 自检
  err = run_test();
  if (err) {
    return HAL_ERROR;
  }
  // 使能DMP
  err = mpu_set_dmp_state(1);
  if (err) {
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Sampling(void) {
  // DMP不需要采样
}

HAL_StatusTypeDef MPU6050_Read_Angle(float dt) {
  UNUSED(dt);
  int err;
  const float Q30 = 1 << 30;
  uint32_t Timestamp;
  int16_t Sensors;
  uint8_t more;
  GW_Angle* a = &GWS_Angle;
  err = dmp_read_fifo(GWS_Angle.MPU_Gyro, GWS_Angle.MPU_Accel,
                      GWS_Angle.MPU_Quat, &Timestamp, &Sensors, &more);
  if (err) {
    return HAL_ERROR;
  }
  if (!(Sensors & INV_WXYZ_QUAT)) {
    return HAL_ERROR;
  }
  Quaternion NumQ;
  NumQ.Q0 = GWS_Angle.MPU_Quat[0] / Q30;
  NumQ.Q1 = GWS_Angle.MPU_Quat[1] / Q30;
  NumQ.Q2 = GWS_Angle.MPU_Quat[2] / Q30;
  NumQ.Q3 = GWS_Angle.MPU_Quat[3] / Q30;
  GWS_Angle.Quat = NumQ;
  // 通过四元数计算欧拉角
  GWS_Angle.Pitch =
      asinf(-2.f * (NumQ.Q1 * NumQ.Q3 + NumQ.Q0 * NumQ.Q2)) * 57.3f;
  GWS_Angle.Roll = atan2(                                                     //
                       2.f * (NumQ.Q2 * NumQ.Q3 + NumQ.Q0 * NumQ.Q1),         //
                       -2.f * (NumQ.Q1 * NumQ.Q1 + NumQ.Q2 * NumQ.Q2) + 1) *  //
                   57.3f;                                                     //
  GWS_Angle.Yaw = atan2(                                                      //
                      2.f * (NumQ.Q1 * NumQ.Q2 + NumQ.Q0 * NumQ.Q3),          //
                      NumQ.Q0 * NumQ.Q0 + NumQ.Q1 * NumQ.Q1 -                 //
                          NumQ.Q2 * NumQ.Q2 - NumQ.Q3 * NumQ.Q3) *            //
                  57.3f;
  return HAL_OK;
}
#else  // 软件解算
#include "i2c.h"
#include "kalman.h"
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
  // FLASH_write(offsets, 6);
  UNUSED(offsets);
  return HAL_OK;
}

#endif