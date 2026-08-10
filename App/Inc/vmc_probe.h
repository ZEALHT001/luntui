#ifndef VMC_PROBE_H
#define VMC_PROBE_H

#include "can.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VMC_WHEEL_LEFT_ID         0x201U
#define VMC_WHEEL_RIGHT_ID        0x202U

#define VMC_JOINT_RIGHT_BACK_FEEDBACK_ID   0x003U
#define VMC_JOINT_RIGHT_AHEAD_FEEDBACK_ID  0x001U
#define VMC_JOINT_LEFT_AHEAD_FEEDBACK_ID   0x007U
#define VMC_JOINT_LEFT_BACK_FEEDBACK_ID    0x005U

#define VMC_JOINT_RIGHT_BACK_CMD_ID        0x004U
#define VMC_JOINT_RIGHT_AHEAD_CMD_ID       0x002U
#define VMC_JOINT_LEFT_AHEAD_CMD_ID        0x008U
#define VMC_JOINT_LEFT_BACK_CMD_ID         0x006U

#define VMC_JOINT_RIGHT_AHEAD_CAN (&hcan1)
#define VMC_JOINT_LEFT_AHEAD_CAN  (&hcan2)
#define VMC_JOINT_RIGHT_BACK_CAN  (&hcan1)
#define VMC_JOINT_LEFT_BACK_CAN   (&hcan2)

#define VMC_ANGLE_T               8191.0f
#define VMC_DM_J8009P_PMAX        12.5f
#define VMC_DM_J8009P_PMIN       -12.5f
#define VMC_DM_J8009_VMAX         45.0f
#define VMC_DM_J8009_VMIN        -45.0f
#define VMC_DM_J8009_TMAX         54.0f
#define VMC_DM_J8009_TMIN        -54.0f

/* Joint zero offsets used before VMC forward kinematics. Tune these after
 * placing the legs in the mechanical reference pose. */
#define VMC_ZERO_LEFT_BACK         0.32942169f
#define VMC_ZERO_LEFT_AHEAD       -4.15347418f
#define VMC_ZERO_RIGHT_AHEAD       4.59674772f
#define VMC_ZERO_RIGHT_BACK        4.56834347f

typedef struct {
  int16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
  float torque;
  float speed_rads;
  float angle;
  int32_t round;
  float last_angle;
  uint32_t last_update_ms;
} vmc_wheel_motor_t;

typedef struct {
  uint8_t id;
  uint8_t status;
  uint8_t t_mos;
  uint8_t t_rotor;
  float position;
  float speed;
  float torque;
  uint32_t last_update_ms;
} vmc_joint_motor_t;

typedef struct {
  double phi1;
  double phi4;
  double length;
  double phi0;
} vmc_leg_state_t;

typedef struct {
  vmc_wheel_motor_t wheel_left;
  vmc_wheel_motor_t wheel_right;
  vmc_joint_motor_t joint_right_back;
  vmc_joint_motor_t joint_right_ahead;
  vmc_joint_motor_t joint_left_ahead;
  vmc_joint_motor_t joint_left_back;
  vmc_leg_state_t left_leg;
  vmc_leg_state_t right_leg;
  uint32_t can_rx_count;
  uint32_t can1_rx_count;
  uint32_t can2_rx_count;
  uint32_t unknown_rx_count;
  uint32_t last_can1_rx_id;
  uint32_t last_can2_rx_id;
  uint32_t mit_zero_tx_count;
  uint32_t mit_zero_tx_fail_count;
} vmc_probe_state_t;

extern volatile vmc_probe_state_t vmc_probe_state;

void vmc_probe_task(void const *argument);
void vmc_can_init(void);
void vmc_can_poll_joints_zero(void);
void vmc_update_leg_state(void);

#ifdef __cplusplus
}
#endif

#endif
