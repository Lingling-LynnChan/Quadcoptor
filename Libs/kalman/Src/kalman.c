#include "kalman.h"

void GW_Kalman_Filter_V1(GW_Kalman_Filter* filter, float in) {
  filter->NowP = filter->LastP + filter->Q;
  filter->Kg = filter->NowP / (filter->NowP + filter->R);
  filter->Out = filter->Out + filter->Kg * (in - filter->Out);
  filter->LastP = (1 - filter->Kg) * filter->NowP;
}