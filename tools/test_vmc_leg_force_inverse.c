#include "vmc_math.h"

#include <math.h>
#include <stdio.h>

int main(void)
{
  static const double phi1[] = {1.45, 1.70, 2.05, 2.35, 2.55};
  static const double phi4[] = {0.45, 0.65, 0.85, 1.05, 1.25};
  static const double force[] = {-300.0, -80.0, 0.0, 80.0, 300.0};
  static const double torque[] = {-100.0, -12.0, 0.0, 12.0, 100.0};
  size_t i;
  size_t j;
  size_t k;
  size_t n;

  for (i = 0U; i < sizeof(phi1) / sizeof(phi1[0]); i++) {
    for (j = 0U; j < sizeof(phi4) / sizeof(phi4[0]); j++) {
      for (k = 0U; k < sizeof(force) / sizeof(force[0]); k++) {
        for (n = 0U; n < sizeof(torque) / sizeof(torque[0]); n++) {
          double joint_torque[2];
          double recovered_force;
          double recovered_torque;
          int ok;

          vmc_leg_force(force[k], torque[n], phi1[i], phi4[j], joint_torque);
          ok = (int)vmc_leg_virtual_force(joint_torque, phi1[i], phi4[j],
                                          &recovered_force,
                                          &recovered_torque);
          if (ok == 0 ||
              fabs(recovered_force - force[k]) > 1.0e-6 ||
              fabs(recovered_torque - torque[n]) > 1.0e-6) {
            fprintf(stderr,
                    "inverse failed at phi1=%.3f phi4=%.3f F=%.3f Tp=%.3f "
                    "-> %.6f %.6f\n",
                    phi1[i], phi4[j], force[k], torque[n],
                    recovered_force, recovered_torque);
            return 1;
          }
        }
      }
    }
  }

  puts("vmc_leg_force inverse round-trip OK");
  return 0;
}
