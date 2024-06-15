#include "spi.h"

#define SCK_H() HAL_GPIO_WritePin(SPI_SCK_GPIO_Port, SPI_SCK_Pin, GPIO_PIN_SET)
#define SCK_L() \
  HAL_GPIO_WritePin(SPI_SCK_GPIO_Port, SPI_SCK_Pin, GPIO_PIN_RESET)
#define MOSI_H() \
  HAL_GPIO_WritePin(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin, GPIO_PIN_SET)
#define MOSI_L() \
  HAL_GPIO_WritePin(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin, GPIO_PIN_RESET)
#define MISO_RD() HAL_GPIO_ReadPin(SPI_MISO_GPIO_Port, SPI_MISO_Pin)

uint8_t GW_SPI_RW(uint8_t data) {
  uint8_t temp = 0x00;
  for (uint8_t i = 0; i < 8; i++) {
    if (data & 0x80) {
      MOSI_H();
    } else {
      MOSI_L();
    }
    data <<= 1;
    temp <<= 1;
    SCK_H();
    if (MISO_RD()) {
      temp++;
    }
    SCK_L();
  }
  return temp;
}