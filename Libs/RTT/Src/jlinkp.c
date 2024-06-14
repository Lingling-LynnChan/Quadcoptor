#include "jlinkp.h"

#include <math.h>

static const float PRECISION = 0.000001f;

char* ftoa(float n) {
  static char s[32];
  s[0] = 0;
  if (isnan(n)) {
    strcpy(s, "nan");
  } else if (isinf(n)) {
    strcpy(s, "inf");
  } else if (n == 0.0) {
    strcpy(s, "0");
  } else {
    int digit, m, m1;
    char* c = s;
    int neg = (n < 0);
    if (neg)
      n = -n;
    m = log10f(n);
    int useExp = (m >= 14 || (neg && m >= 9) || m <= -9);
    if (neg)
      *(c++) = '-';
    if (useExp) {
      if (m < 0)
        m -= 1.0;
      n = n / powf(10.0, m);
      m1 = m;
      m = 0;
    }
    if (m < 1.0) {
      m = 0;
    }
    while (n > PRECISION || m >= 0) {
      double weight = powf(10.0, m);
      if (weight > 0 && !isinf(weight)) {
        digit = floorf(n / weight);
        n -= (digit * weight);
        *(c++) = '0' + digit;
      }
      if (m == 0 && n > 0)
        *(c++) = '.';
      m--;
    }
    if (useExp) {
      int i, j;
      *(c++) = 'e';
      if (m1 > 0) {
        *(c++) = '+';
      } else {
        *(c++) = '-';
        m1 = -m1;
      }
      m = 0;
      while (m1 > 0) {
        *(c++) = '0' + m1 % 10;
        m1 /= 10;
        m++;
      }
      c -= m;
      for (i = 0, j = m - 1; i < j; i++, j--) {
        c[i] ^= c[j];
        c[j] ^= c[i];
        c[i] ^= c[j];
      }
      c += m;
    }
    *(c) = '\0';
  }
  return s;
}