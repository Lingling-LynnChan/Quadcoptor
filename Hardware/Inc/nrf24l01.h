#ifndef __GW_NRF24L01_H__
#define __GW_NRF24L01_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void NRF24L01_Init(void);
HAL_StatusTypeDef NRF24L01_USE_RX(void);
HAL_StatusTypeDef NRF24L01_USE_TX(void);
HAL_StatusTypeDef NRF24L01_Send_Reg(uint8_t addr, uint8_t data);
HAL_StatusTypeDef NRF24L01_Read_Reg(uint8_t addr, uint8_t* data);
HAL_StatusTypeDef NRF24L01_Send_Data(uint8_t addr, uint8_t* data, uint8_t len);
HAL_StatusTypeDef NRF24L01_Read_Data(uint8_t addr, uint8_t* data, uint8_t len);
HAL_StatusTypeDef NRF24L01_TxPacket(uint8_t *txbuf);
HAL_StatusTypeDef NRF24L01_RxPacket(uint8_t *rxbuf);

#ifdef __cplusplus
}
#endif

#endif /*__GW_NRF24L01_H__*/
