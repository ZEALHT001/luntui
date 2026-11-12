#ifndef VMC_LESO_H
#define VMC_LESO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VMC_LESO_PLANT_STATES 6U
#define VMC_LESO_INPUT_COUNT 2U
#define VMC_LESO_STATE_COUNT \
  (VMC_LESO_PLANT_STATES + VMC_LESO_INPUT_COUNT)
#define VMC_LESO_MODEL_POINTS 9U

/* State order matches the plant model: theta, dtheta, x, dx, phi, dphi,
 * followed by one disturbance per input. */
typedef struct {
  float leg_length;
  float a[VMC_LESO_STATE_COUNT][VMC_LESO_STATE_COUNT];
  float b[VMC_LESO_STATE_COUNT][VMC_LESO_INPUT_COUNT];
  float l[VMC_LESO_STATE_COUNT][VMC_LESO_PLANT_STATES];
} vmc_leso_model_point_t;

extern const vmc_leso_model_point_t
    vmc_leso_model_points[VMC_LESO_MODEL_POINTS];
extern const float vmc_leso_sample_time_s;

void vmc_leso_reset(void);
void vmc_leso_set_enable(uint8_t enable);
uint8_t vmc_leso_is_enabled(void);
/* Compensation depth, 0..1, so the observer can be faded in. */
void vmc_leso_set_compensation(float compensation);
/* Per-channel cap on the applied correction. */
void vmc_leso_set_limit(float wheel_limit, float leg_limit);

/* Advance the observer one control period and return the disturbance
 * estimate in input units (wheel torque, model-convention leg force). */
void vmc_leso_update(float leg_length,
                     const float measurement[VMC_LESO_PLANT_STATES],
                     const float input[VMC_LESO_INPUT_COUNT],
                     float disturbance[VMC_LESO_INPUT_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
