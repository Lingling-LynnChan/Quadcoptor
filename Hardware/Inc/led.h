#ifndef __GW_LED_H__
#define __GW_LED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum {
  GW_LED_OFF,   // 全灭
  GW_LED_ON,    // 全亮
  GW_LED_FAST,  // 快闪
  GW_LED_SLOW,  // 慢闪
} GW_LED;

extern GW_LED GWS_LED;

void GW_LED_Status(void);

#ifdef __cplusplus
}
#endif

#endif /*__GW_LED_H__ */