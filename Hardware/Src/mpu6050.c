#include "mpu6050.h"
#include <math.h>
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
  // 设置采样率
  err = mpu_set_sample_rate(200);  // 200Hz: 5ms
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

HAL_StatusTypeDef MPU6050_DMP_Read(GW_Angle* a) {
  int err;
  const float Q30 = 1 << 30;
  uint32_t Timestamp;
  int16_t Sensors;
  uint8_t more;
  err = dmp_read_fifo(a->MPU_Gyro, a->MPU_Accel, a->MPU_Quat, &Timestamp,
                      &Sensors, &more);
  if (err) {
    return HAL_ERROR;
  }
  if (!(Sensors & INV_WXYZ_QUAT)) {
    return HAL_ERROR;
  }
  Quaternion NumQ;
  NumQ.Q0 = a->MPU_Quat[0] / Q30;
  NumQ.Q1 = a->MPU_Quat[1] / Q30;
  NumQ.Q2 = a->MPU_Quat[2] / Q30;
  NumQ.Q3 = a->MPU_Quat[3] / Q30;
  a->Quat = NumQ;
  // 通过四元数计算欧拉角
  a->Pitch = asinf(-2.f * (NumQ.Q1 * NumQ.Q3 + NumQ.Q0 * NumQ.Q2)) * 57.3f;
  a->Roll = atan2(                                                     //
                2.f * (NumQ.Q2 * NumQ.Q3 + NumQ.Q0 * NumQ.Q1),         //
                -2.f * (NumQ.Q1 * NumQ.Q1 + NumQ.Q2 * NumQ.Q2) + 1) *  //
            57.3f;                                                     //
  a->Yaw = atan2(                                                      //
               2.f * (NumQ.Q1 * NumQ.Q2 + NumQ.Q0 * NumQ.Q3),          //
               NumQ.Q0 * NumQ.Q0 + NumQ.Q1 * NumQ.Q1 -                 //
                   NumQ.Q2 * NumQ.Q2 - NumQ.Q3 * NumQ.Q3) *            //
           57.3f;
  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_GW_Read(GW_Angle* a, uint32_t dt) {
  if (mpu_get_gyro_reg(a->MPU_Gyro, NULL)) {
    return HAL_ERROR;
  }
  if (mpu_get_accel_reg(a->MPU_Accel, NULL)) {
    return HAL_ERROR;
  }
  // TODO 姿态解算
  return HAL_OK;
}