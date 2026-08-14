#ifndef VMC_LQR_H
#define VMC_LQR_H

#ifdef __cplusplus
extern "C" {
#endif

void vmc_lqr_update_gain(double leg_length, double k[12]);

#ifdef __cplusplus
}
#endif

#endif
