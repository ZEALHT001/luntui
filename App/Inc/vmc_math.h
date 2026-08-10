#ifndef VMC_MATH_H
#define VMC_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#define VMC_PI 3.14159265358979323846f

float vmc_rad_format(float angle);
void vmc_leg_position(double phi1, double phi4, double *length, double *phi0);
void vmc_leg_speed(double d_phi1, double d_phi4, double phi1, double phi4,
                   double *d_length, double *d_phi0);
void vmc_leg_force(double force, double torque_phi0, double phi1, double phi4,
                   double joint_torque[2]);

#ifdef __cplusplus
}
#endif

#endif
