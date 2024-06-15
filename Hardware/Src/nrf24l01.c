#include "nrf24l01.h"
#include "spi.h"

// 配对密码
uint8_t NRF24L01_PAIR[] = {0xE1, 0xE2, 0xE3, 0xE4, 0xE5};

#define NRF24L01_CE_L() \
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET)
#define NRF24L01_CE_H() \
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET)
#define NRF24L01_CSN_L() \
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_RESET)
#define NRF24L01_CSN_H() \
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_SET)
#define SPI_IRQ_RD() HAL_GPIO_ReadPin(NRF24L01_IRQ_GPIO_Port, NRF24L01_IRQ_Pin)

// NRF24L01发送接收数据宽度定义
// 5字节的地址宽度
#define TX_ADR_WIDTH 5
// 5字节的地址宽度
#define RX_ADR_WIDTH 5
// 20字节的用户数据宽度
#define TX_PLOAD_WIDTH 32
// 20字节的用户数据宽度
#define RX_PLOAD_WIDTH 32
// NRF24L01寄存器操作命令
// 读配置寄存器,低5位为寄存器地址
#define SPI_READ_REG 0x00
// 写配置寄存器,低5位为寄存器地址
#define SPI_WRITE_REG 0x20
// 读RX有效数据,1~32字节
#define RD_RX_PLOAD 0x61
// 写TX有效数据,1~32字节
#define WR_TX_PLOAD 0xA0
// 清除TX FIFO寄存器.发射模式下用
#define FLUSH_TX 0xE1
// 清除RX FIFO寄存器.接收模式下用
#define FLUSH_RX 0xE2
// 重新使用上一包数据,CE为高,数据包被不断发送.
#define REUSE_TX_PL 0xE3
// 空操作,可以用来读状态寄存器
#define NOP 0xFF
// SPI(NRF24L01)寄存器地址
// 配置寄存器地址
// bit0:1接收模式,0发射模式
// bit1:电选择
// bit2:CRC模式
// bit3:CRC使能
// bit4:中断MAX_RT(达到最大重发次数中断)使能
// bit5:中断TX_DS使能
// bit6:中断RX_DR使能
#define NCONFIG 0x00
// 使能自动应答功能 bit0 ~ 5,对应通道0 ~ 5
#define EN_AA 0x01
// 接收地址允许,bit0 ~ 5,对应通道0 ~ 5
#define EN_RXADDR 0x02
// 设置地址宽度(所有数据通道)
// bit1,0:00,3字节;01,4字节;02,5字节;
#define SETUP_AW 0x03
// 建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
#define SETUP_RETR 0x04
// RF通道,bit6:0,工作通道频率;
#define RF_CH 0x05
// RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
#define RF_SETUP 0x06
// 状态寄存器
// bit0:TX FIFO满标志
// bit[3:1]:1,接收数据通道号(最大:6)
// bit4:达到最多次重发
// bit5:数据发送完成中断
// bit6:接收数据中断
#define STATUS 0x07
// 达到最大发送次数中断
#define MAX_TX 0x10
// TX发送完成中断
#define TX_OK 0x20
// 接收到数据中断
#define RX_OK 0x40
// 发送检测寄存器,bit7:4,数据包丢失计数器;bit3:0,重发计数器
#define OBSERVE_TX 0x08
// 载波检测寄存器,bit0,载波检测;
#define CD 0x09
// 数据通道0接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P0 0x0A
// 数据通道1接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P1 0x0B
// 数据通道2接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P2 0x0C
// 数据通道3接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P3 0x0D
// 数据通道4接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P4 0x0E
// 数据通道5接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P5 0x0F
// 发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等
#define TX_ADDR 0x10
// 接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P0 0x11
// 接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1 0x12
// 接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2 0x13
// 接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3 0x14
// 接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4 0x15
// 接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5 0x16
// FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX
// FIFO满标志;bit2,3,保留 bit4,TX FIFO空标志;bit5,TX
// FIFO满标志;bit6,1,循环发送上一数据包.0,不循环;
#define FIFO_STATUS 0x17

static HAL_StatusTypeDef NRF24L01_Check(void) {
  uint8_t writes[5] = {0xA1, 0xB2, 0xC3, 0xD4, 0xE5};
  uint8_t reads[5];
  HAL_StatusTypeDef hs;
  hs = NRF24L01_Send_Data(TX_ADDR, writes, 5);
  if (hs != HAL_OK) {
    jlink("NRF24L01: Send Data Error\n");
  }
  hs = NRF24L01_Read_Data(TX_ADDR, reads, 5);
  if (hs != HAL_OK) {
    jlink("NRF24L01: Read Data Error\n");
  }
  for (uint8_t i = 0; i < 5; i++) {
    if (reads[i] != writes[i]) {
      jlink("NRF24L01: data[%u] = %02x\n", i, reads[i]);
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

void NRF24L01_Init(void) {
  NRF24L01_CE_L();
  NRF24L01_CSN_H();
  HAL_Delay(100);
  NRF24L01_Set_Rx();
  while (NRF24L01_Check() != HAL_OK) {
    jlink("NRF24L01: Try To Use RxMode Again\n");
    NRF24L01_Set_Rx();
  }
  jlink("NRF24L01 Init Success\n");
}

// 接收模式
HAL_StatusTypeDef NRF24L01_Set_Rx(void) {
  HAL_StatusTypeDef hs = HAL_OK;
  NRF24L01_CE_L();  // 发射使能
  // 设置目标地址
  hs |= NRF24L01_Send_Data(RX_ADDR_P0, NRF24L01_PAIR, RX_ADR_WIDTH);
  // 启用通道0的自动应答功能
  hs |= NRF24L01_Send_Reg(EN_AA, 0x01);
  // 启用通道0的接收地址
  hs |= NRF24L01_Send_Reg(EN_RXADDR, 0x01);
  // 设置通信频率
  hs |= NRF24L01_Send_Reg(RF_CH, 0x01);
  // 设置通道0的有效数据宽度
  hs |= NRF24L01_Send_Reg(RX_PW_P0, RX_PLOAD_WIDTH);
  // 设置TX发射参数,0db增益,2Mbps,低噪声增益开启
  hs |= NRF24L01_Send_Reg(RF_SETUP, 0x07);
  // 设置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX接收模式
  hs |= NRF24L01_Send_Reg(NCONFIG, 0x0f);
  NRF24L01_CE_H();  // 发射失能
  return hs;
}

// 发射模式
HAL_StatusTypeDef NRF24L01_Set_Tx(void) {
  // 不需要发射
  return HAL_OK;
}

HAL_StatusTypeDef NRF24L01_Send_Reg(uint8_t addr, uint8_t data) {
  NRF24L01_CSN_L();
  GW_SPI_RW(SPI_WRITE_REG | addr);
  GW_SPI_RW(data);
  NRF24L01_CSN_H();
  return HAL_OK;
}

HAL_StatusTypeDef NRF24L01_Send_Data(uint8_t addr, uint8_t* data, uint8_t len) {
  NRF24L01_CSN_L();
  GW_SPI_RW(SPI_WRITE_REG | addr);
  for (uint8_t i = 0; i < len; i++) {
    GW_SPI_RW(data[i]);
  }
  NRF24L01_CSN_H();
  return HAL_OK;
}

HAL_StatusTypeDef NRF24L01_Read_Reg(uint8_t addr, uint8_t* data) {
  NRF24L01_CSN_L();
  GW_SPI_RW(SPI_READ_REG | addr);
  *data = GW_SPI_RW(NOP);
  NRF24L01_CSN_H();
  return HAL_OK;
}

HAL_StatusTypeDef NRF24L01_Read_Data(uint8_t addr, uint8_t* data, uint8_t len) {
  NRF24L01_CSN_L();
  GW_SPI_RW(SPI_READ_REG | addr);
  for (uint8_t i = 0; i < len; i++) {
    data[i] = GW_SPI_RW(NOP);
  }
  NRF24L01_CSN_H();
  return HAL_OK;
}
HAL_StatusTypeDef NRF24L01_TxPacket(uint8_t* txbuf) {
  // 不需要发射数据
  return HAL_ERROR;
}
HAL_StatusTypeDef NRF24L01_RxPacket(uint8_t* rxbuf) {
  uint8_t status;
  NRF24L01_Read_Reg(STATUS, &status);
  NRF24L01_Send_Reg(STATUS, status);
  if (status & RX_OK) {
    NRF24L01_Read_Data(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);
    NRF24L01_Send_Reg(FLUSH_RX, 0xff);
    return HAL_OK;
  }
  return HAL_ERROR;
}