#include "led.h"
#include "tim.h"

#define LED_H(X) ((TIM3->CCR##X) = 1000)  // 灭
#define LED_L(X) ((TIM3->CCR##X) = 500)   // 亮
#define LED_TOGGLE(X) ((TIM3->CCR##X) = ((TIM3->CCR##X) == 500) ? 1000 : 500)
#define FAST_TICK 100
#define SLOW_TICK 200

GW_LED GWS_LED = GW_LED_FAST;

void GW_LED_Status(void) {
  static uint32_t lastTime = 0;
  if (lastTime == 0) {
    lastTime = GW_SYS_MS;
  }
  if (GW_SYS_MS - lastTime < (GWS_LED == GW_LED_FAST ? FAST_TICK : SLOW_TICK)) {
    return;
  }
  switch (GWS_LED) {
    case GW_LED_OFF:
      LED_L(1);
      LED_L(2);
      LED_L(3);
      LED_L(4);
      break;
    case GW_LED_ON:
      LED_H(1);
      LED_H(2);
      LED_H(3);
      LED_H(4);
      break;
    case GW_LED_FAST:
      goto flash;
    case GW_LED_SLOW:
    flash:
      LED_TOGGLE(1);
      LED_TOGGLE(2);
      LED_TOGGLE(3);
      LED_TOGGLE(4);
      break;
    default:
      break;
  }
  lastTime = GW_SYS_MS;
}