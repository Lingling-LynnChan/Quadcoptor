#ifndef __KALMAN_H__
#define __KALMAN_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float LastP;
  float NowP;
  float Out;
  float Kg;
  float Q;
  float R;
} GW_Kalman_Filter;

void GW_Kalman_Filter_V1(GW_Kalman_Filter* filter, float in);

#ifdef __cplusplus
}
#endif

#endif /*__KALMAN_H__ */