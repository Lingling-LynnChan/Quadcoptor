#include "anotc.h"
#include <string.h>

// 发送PID校验
int16_t GWS_PID_Check;

// 临时保存上位机发过来的PID数据 防止数据来不及保存而被下一组覆盖
static uint8_t RatePID[18];
static uint8_t AnglePID[18];
static uint8_t HighPID[18];

// 接收到上位机的数据种类标志位
static struct {
  uint8_t PID1 : 1;           // 接受到上位机PID组1
  uint8_t PID2 : 1;           // 接受到上位机PID组2
  uint8_t PID3 : 1;           // 接受到上位机PID组3
  uint8_t PID4 : 1;           // 接受到上位机PID组4
  uint8_t PID5 : 1;           // 接受到上位机PID组5
  uint8_t PID6 : 1;           // 接受到上位机PID组6
  uint8_t CMD2_READ_PID : 1;  // 接受到上位机读取PID的请求
} ANOTC_Recived_flag;

// 处理接收到的数据帧
void ANOTC_Recive(int8_t* pt) {
  switch (pt[2]) {
    case ANOTC_PID1:  // 角速度环PID
      ANOTC_Recived_flag.PID1 = 1;
      memcpy(RatePID, &pt[4], 18);
      break;
    case ANOTC_PID2:  // 角度环PID
      memcpy(AnglePID, &pt[4], 18);
      ANOTC_Recived_flag.PID2 = 1;
      break;
    case ANOTC_PID3:  // 高度环PID
      memcpy(HighPID, &pt[4], 18);
      ANOTC_Recived_flag.PID3 = 1;
      break;
    case ANOTC_PID4:
      break;
    case ANOTC_PID5:
      break;
    case ANOTC_PID6:
      break;
    case 0x01:  // CMD1
      break;
    case 0x02: {  // CMD2
      enum {
        READ_PID = 0X01,           // 读取飞控的PID请求
        READ_MODE = 0x02,          // 读取飞行模式
        READ_ROUTE = 0x21,         // 读取航点信息
        READ_VERSION = 0XA0,       // 读取飞控版本
        RETURN_DEFAULT_PID = 0xA1  // 恢复默认PID
      };
      switch (*(uint8_t*)&pt[4])  // 判断上位机发来CMD的内容
      {
        case READ_PID:  // 上位机请求读取飞控PID数据
          ANOTC_Recived_flag.CMD2_READ_PID = 1;
          break;
        case READ_MODE:
          break;
        case READ_ROUTE:
          break;
        case READ_VERSION:
          break;
        case RETURN_DEFAULT_PID:
          break;
        default:
          break;
      }

    } break;
    case ANOTC_RCDATA:  // 遥控器数据
      break;

    default:
      break;
  }
  return;
}
// 发送数据给上位机
void ANOTC_Send(ANOTC_SEND type) {
  uint8_t len = 2;
  int16_t Ano[12];
  int8_t* pt = (int8_t*)(Ano);

  switch (type) {
    case ANOTC_PID1:
      goto TO_PID;
    case ANOTC_PID2:
      goto TO_PID;
    case ANOTC_PID3:
      goto TO_PID;
    case ANOTC_PID4:
      goto TO_PID;
    case ANOTC_PID5:
      goto TO_PID;
    case ANOTC_PID6:
    TO_PID:
      break;
    case ANOTC_MOTOR:
      break;
    case ANOTC_RCDATA:
      break;
    case ANOTC_MPU_MAGIC:
      break;
    case ANOTC_SENSER2:
      break;
    case ANOTC_STATUS:
      break;
    case ANOTC_POWER:
      break;
    case ANOTC_CHECK:
      Ano[2] = GWS_PID_Check;
      len = 2;
      break;
    default:
      break;
  }
  uint16_t AAAA = 0xAAAA;
  Ano[0] = *(int16_t*)&AAAA;
  Ano[1] = len | type << 8;
  pt[len + 4] = (int8_t)(0xAA + 0xAA);
  for (uint8_t i = 2; i < len + 4; i += 2) {
    pt[i] ^= pt[i + 1];
    pt[i + 1] ^= pt[i];
    pt[i] ^= pt[i + 1];
    pt[len + 4] += pt[i] + pt[i + 1];
  }
  CDC_Transmit_FS((uint8_t*)pt, len + 5);
}
// 轮询发送到上位机
void ANOTC_Polling(void) {}
