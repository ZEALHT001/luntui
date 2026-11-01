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

void vmc_leg_speed(double d_phi1, double d_phi4, double phi1, double phi4,
                   double *d_length, double *d_phi0)
{
  double t10_tmp;
  double t12;
  double t13;
  double t14_tmp;
  double t16;
  double t17;
  double t2;
  double t29;
  double t3;
  double t30;
  double t31;
  double t32;
  double t37;
  double t38;
  double t39;
  double t4;
  double t40;
  double t5;
  double t51;
  double t57;
  double t58;
  double t63;
  double t66;
  double t81;

  t2 = cos(phi1);
  t3 = cos(phi4);
  t4 = sin(phi1);
  t5 = sin(phi4);
  t10_tmp = t2 * 0.21;
  t12 = t2 * 0.105;
  t13 = t3 * 0.105;
  t14_tmp = t4 * 0.21;
  t16 = t4 * 0.105;
  t17 = t5 * 0.105;
  t29 = t10_tmp - t3 * 0.21;
  t30 = t12 - t13;
  t31 = t14_tmp - t5 * 0.21;
  t32 = t16 - t17;
  t37 = t4 * t29 * 0.42;
  t38 = t5 * t29 * 0.42;
  t39 = t2 * t31 * 0.42;
  t40 = t3 * t31 * 0.42;
  t51 = t29 * t29 + t31 * t31;
  t57 = 1.0 / ((t13 - t12) + t51);
  t58 = t57 * t57;
  t2 = sqrt((t30 * t30 + t32 * t32) - t51 * t51);
  t63 = 1.0 / t2;
  t66 = (t17 - t16) + t2;
  t2 = atan(t57 * t66) * 2.0;
  t29 = cos(t2);
  t31 = sin(t2);
  t81 = 1.0 / (t58 * (t66 * t66) + 1.0);
  t16 = t58 * ((t16 + t39) - t37) * t66 +
        t57 * (t12 - t63 *
                         ((t10_tmp * t32 + t4 * t30 * -0.21) +
                          t51 * (t37 - t39) * 2.0) /
                         2.0);
  t2 = t58 * ((t17 + t40) - t38) * t66 +
       t57 * (t13 - t63 *
                        ((t5 * t30 * -0.21 + t51 * (t38 - t40) * 2.0) +
                         t3 * t32 * 0.21) /
                        2.0);
  t39 = t10_tmp + t29 / 4.0;
  t4 = t14_tmp + t31 / 4.0;
  t51 = t29 * t81;
  t12 = t51 * t2;
  t31 *= t81;
  t57 = t31 * t2;
  t58 = t39 * t39;
  t63 = 1.0 / t39;
  t29 = t4 * t4 + t58;
  t2 = 1.0 / sqrt(t29);
  t66 = 1.0 / t29;
  t51 = t10_tmp - t51 * t16 / 2.0;
  t29 = t14_tmp - t31 * t16 / 2.0;
  *d_length = d_phi4 * t2 * (t4 * t12 - t39 * t57) / 2.0 +
              d_phi1 * t2 * (t4 * t51 * 2.0 - t39 * t29 * 2.0) / 2.0;
  t2 = t4 * (1.0 / t58);
  *d_phi0 = d_phi1 * t58 * t66 * (t63 * t51 + t2 * t29) -
            d_phi4 * t58 * t66 * (t63 * (0.0 - t12 / 2.0) - t2 * (t57 / 2.0));
}

void vmc_leg_force(double force, double torque_phi0, double phi1, double phi4,
                   double joint_torque[2])
{
  double t110;
  double t112_tmp;
  double t118_tmp;
  double t130;
  double t134_tmp;
  double t141_tmp;
  double t16_tmp;
  double t185;
  double t18_tmp;
  double t19_tmp;
  double t20_tmp;
  double t23_tmp;
  double t49_tmp;
  double t50_tmp;
  double t51_tmp;
  double t52_tmp;
  double t5_tmp;
  double t63_tmp;
  double t64_tmp;
  double t65_tmp;
  double t66_tmp;
  double t6_tmp;
  double t7_tmp;
  double t91_tmp;

  t130 = cos(phi1);
  t5_tmp = cos(phi4);
  t6_tmp = sin(phi1);
  t7_tmp = sin(phi4);
  t16_tmp = t130 * 0.21;
  t18_tmp = t130 * 0.105;
  t19_tmp = t5_tmp * 0.105;
  t20_tmp = t6_tmp * 0.21;
  t185 = t6_tmp * 0.105;
  t23_tmp = t7_tmp * 0.105;
  t49_tmp = t16_tmp - t5_tmp * 0.21;
  t50_tmp = t18_tmp - t19_tmp;
  t51_tmp = t20_tmp - t7_tmp * 0.21;
  t52_tmp = t185 - t23_tmp;
  t63_tmp = t6_tmp * t49_tmp * 0.42;
  t64_tmp = t7_tmp * t49_tmp * 0.42;
  t65_tmp = t130 * t51_tmp * 0.42;
  t66_tmp = t5_tmp * t51_tmp * 0.42;
  t91_tmp = t49_tmp * t49_tmp + t51_tmp * t51_tmp;
  t49_tmp = 1.0 / ((t19_tmp - t18_tmp) + t91_tmp);
  t51_tmp = t49_tmp * t49_tmp;
  t110 = sqrt((t50_tmp * t50_tmp + t52_tmp * t52_tmp) - t91_tmp * t91_tmp);
  t112_tmp = 1.0 / t110;
  t118_tmp = (t23_tmp - t185) + t110;
  t130 = atan(t49_tmp * t118_tmp) * 2.0;
  t110 = cos(t130);
  t134_tmp = sin(t130);
  t141_tmp = 1.0 / (t51_tmp * (t118_tmp * t118_tmp) + 1.0);
  t63_tmp = t51_tmp * ((t185 + t65_tmp) - t63_tmp) * t118_tmp +
            t49_tmp * (t18_tmp - t112_tmp *
                                     ((t16_tmp * t52_tmp + t6_tmp * t50_tmp * -0.21) +
                                      t91_tmp * (t63_tmp - t65_tmp) * 2.0) /
                                     2.0);
  t65_tmp = t51_tmp * ((t23_tmp + t66_tmp) - t64_tmp) * t118_tmp +
            t49_tmp * (t19_tmp - t112_tmp *
                                     ((t7_tmp * t50_tmp * -0.21 +
                                       t91_tmp * (t64_tmp - t66_tmp) * 2.0) +
                                      t5_tmp * t52_tmp * 0.21) /
                                     2.0);
  t6_tmp = t16_tmp + t110 / 4.0;
  t18_tmp = t20_tmp + t134_tmp / 4.0;
  t130 = t6_tmp * t6_tmp;
  t185 = 1.0 / t6_tmp;
  t118_tmp = t18_tmp * t18_tmp + t130;
  t112_tmp = t110 * t141_tmp;
  t91_tmp = t16_tmp - t112_tmp * t63_tmp / 2.0;
  t110 = t134_tmp * t141_tmp;
  t49_tmp = t20_tmp - t110 * t63_tmp / 2.0;
  t51_tmp = force * (1.0 / sqrt(t118_tmp));
  t118_tmp = torque_phi0 * t130 * (1.0 / t118_tmp);
  t130 = t18_tmp * (1.0 / t130);
  joint_torque[0] = t51_tmp * (t18_tmp * t91_tmp * 2.0 - t6_tmp * t49_tmp * 2.0) / 2.0 +
                    t118_tmp * (t185 * t91_tmp + t130 * t49_tmp);
  joint_torque[1] = t51_tmp * (t112_tmp * t18_tmp * t65_tmp - t110 * t6_tmp * t65_tmp) / 2.0 -
                    t118_tmp * (t185 * (0.0 - t112_tmp * t65_tmp / 2.0) -
                                 t130 * (t110 * t65_tmp / 2.0));
}

uint8_t vmc_leg_virtual_force(const double joint_torque[2], double phi1,
                              double phi4, double *force,
                              double *torque_phi0)
{
  double force_basis[2];
  double force_coefficient;

  if (joint_torque == 0 || force == 0 || torque_phi0 == 0) {
    return 0U;
  }

  /* vmc_leg_force() is affine in (F, Tp).  Its documented, posture-independent
   * Tp terms are +0.5 on each joint; the F terms are equal and opposite.
   * Evaluating only the unit-F input is therefore enough to recover both. */
  vmc_leg_force(1.0, 0.0, phi1, phi4, force_basis);
  force_coefficient = force_basis[0];
  if (fabs(force_coefficient) <= 1.0e-9 ||
      fabs(force_basis[1] + force_basis[0]) > 1.0e-9) {
    return 0U;
  }

  *force = (joint_torque[0] - joint_torque[1]) /
           (2.0 * force_coefficient);
  *torque_phi0 = joint_torque[0] + joint_torque[1];
  return 1U;
}
