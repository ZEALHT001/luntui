#ifndef VMC_MATH_H
#define VMC_MATH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VMC_PI 3.14159265358979323846f

float vmc_rad_format(float angle);
void vmc_leg_position(double phi1, double phi4, double *length, double *phi0);
void vmc_leg_speed(double d_phi1, double d_phi4, double phi1, double phi4,
                   double *d_length, double *d_phi0);
void vmc_leg_position_float(float phi1, float phi4, float *length,
                            float *phi0);
void vmc_leg_speed_float(float d_phi1, float d_phi4, float phi1, float phi4,
                         float *d_length, float *d_phi0);
void vmc_leg_force(double force, double torque_phi0, double phi1, double phi4,
                   double joint_torque[2]);
void vmc_leg_force_float(float force, float torque_phi0, float phi1,
                         float phi4, float joint_torque[2]);
uint8_t vmc_leg_virtual_force(const double joint_torque[2], double phi1,
                              double phi4, double *force,
                              double *torque_phi0);
uint8_t vmc_leg_virtual_force_float(const float joint_torque[2], float phi1,
                                    float phi4, float *force,
                                    float *torque_phi0);

#ifdef __cplusplus
}
#endif

#endif
