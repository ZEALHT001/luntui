#include "vmc_math.h"

#include <math.h>
#include <stdint.h>

static double rtNaN;
static double rtInf = INFINITY;
static double rtMinusInf = -INFINITY;

static uint8_t rt_is_nan(double value)
{
  return (value != value) ? 1U : 0U;
}

static uint8_t rt_is_inf(double value)
{
  return (value == rtInf || value == rtMinusInf) ? 1U : 0U;
}

static double safe_atan2(double y, double x)
{
  if (rt_is_nan(y) || rt_is_nan(x)) {
    return rtNaN;
  }

  if (rt_is_inf(y) && rt_is_inf(x)) {
    return atan2((y > 0.0) ? 1.0 : -1.0, (x > 0.0) ? 1.0 : -1.0);
  }

  if (x == 0.0) {
    if (y > 0.0) {
      return 3.14159265358979323846 / 2.0;
    }
    if (y < 0.0) {
      return -(3.14159265358979323846 / 2.0);
    }
    return 0.0;
  }

  return atan2(y, x);
}

float vmc_rad_format(float angle)
{
  const float min_value = -VMC_PI;
  const float max_value = VMC_PI;
  const float range = max_value - min_value;

  while (angle > max_value) {
    angle -= range;
  }

  while (angle < min_value) {
    angle += range;
  }

  return angle;
}

void vmc_leg_position(double phi1, double phi4, double *length, double *phi0)
{
  double a;
  double t10;
  double t12;
  double t13;
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t8;
  double t9;

  t2 = cos(phi1);
  t3 = cos(phi4);
  t4 = sin(phi1);
  t5 = sin(phi4);
  t6 = t2 * 0.21;
  t8 = t2 * 0.105;
  t9 = t3 * 0.105;
  t10 = t4 * 0.21;
  t12 = t4 * 0.105;
  t13 = t5 * 0.105;
  t4 = t6 - t3 * 0.21;
  a = t8 - t9;
  t2 = t10 - t5 * 0.21;
  t3 = t12 - t13;
  t2 = t4 * t4 + t2 * t2;
  t2 = atan(1.0 / ((t9 - t8) + t2) *
            ((t13 - t12) + sqrt((a * a + t3 * t3) - t2 * t2))) *
       2.0;
  t4 = t10 + sin(t2) / 4.0;
  t2 = t6 + cos(t2) / 4.0;
  *length = sqrt(t4 * t4 + t2 * t2);
  *phi0 = safe_atan2(t4, t2);
}
