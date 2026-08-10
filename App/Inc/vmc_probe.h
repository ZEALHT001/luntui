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

#define VMC_LEG_LENGTH_DEFAULT     0.385f
#define VMC_LEG_LENGTH_MIN         0.120f
#define VMC_LEG_LENGTH_MAX         0.385f
#define VMC_LEG_LENGTH_KP_DEFAULT  1.0f
#define VMC_LEG_LENGTH_KD_DEFAULT  0.0f
#define VMC_LEG_FORCE_MAX_DEFAULT  120.0f
#define VMC_LEG_LENGTH_PID_DIRECTION -1.0f

/* Raw joint positions captured with both legs vertical down at 385 mm. */
#define VMC_REF_RAW_LEFT_BACK      2.53891087f
#define VMC_REF_RAW_LEFT_AHEAD    -3.22137070f
#define VMC_REF_RAW_RIGHT_AHEAD    3.66464424f
#define VMC_REF_RAW_RIGHT_BACK     2.35885429f

/* VMC model angles at the same 385 mm reference pose. */
#define VMC_REF_MODEL_PHI1         2.20948918f
#define VMC_REF_MODEL_PHI4         0.93210348f

/*
 * Motor-to-model angle directions.
 * If the measured length increases while the physical leg retracts, flip the
 * corresponding leg direction pair instead of changing the reference raw angle.
 */
#define VMC_DIR_LEFT_BACK         -1.0f
#define VMC_DIR_LEFT_AHEAD        -1.0f
#define VMC_DIR_RIGHT_AHEAD        1.0f
#define VMC_DIR_RIGHT_BACK         1.0f

#define VMC_DM_STATUS_NOT_ENABLED  0x00U
#define VMC_DM_STATUS_CAN_TIMEOUT  0x0DU

#define VMC_DM_STATE_CONTROL       0U
#define VMC_DM_STATE_INIT          1U
#define VMC_DM_STATE_RECOVER       2U
#define VMC_DM_STATE_DISABLE       3U

#define VMC_PHI0_PROTECT_MIN_RAD   (20.0f * VMC_PI / 180.0f)
#define VMC_PHI0_PROTECT_MAX_RAD   (160.0f * VMC_PI / 180.0f)
#define VMC_PHI0_PROTECT_NONE      0U
#define VMC_PHI0_PROTECT_LEFT      1U
#define VMC_PHI0_PROTECT_RIGHT     2U
#define VMC_PHI0_PROTECT_BOTH      3U

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
  double d_phi1;
  double d_phi4;
  double length;
  double phi0;
  double d_length;
  double d_phi0;
  double length_error;
  double length_p_output;
  double length_d_output;
  double length_pid_output;
  double force_command;
  double torque_phi0_command;
  double joint_torque_ahead;
  double joint_torque_back;
} vmc_leg_state_t;

typedef struct {
  uint8_t enabled;
  float target_length;
  float kp_length;
  float kd_length;
  float max_force;
} vmc_control_t;

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
  uint32_t mit_enable_tx_count;
  uint32_t mit_enable_tx_fail_count;
  uint32_t dm_init_tx_count;
  uint32_t dm_init_tx_fail_count;
  uint32_t dm_recover_tx_count;
  uint32_t dm_recover_tx_fail_count;
  uint32_t dm_disable_tx_count;
  uint32_t dm_disable_tx_fail_count;
  uint8_t dm_state;
  uint8_t dm_tx_phase;
  uint8_t phi0_protect_latched;
  uint8_t phi0_protect_source;
} vmc_probe_state_t;

extern volatile vmc_probe_state_t vmc_probe_state;
extern volatile vmc_control_t vmc_control;

void vmc_probe_task(void const *argument);
void vmc_can_init(void);
void vmc_can_poll_joints_zero(void);
void vmc_dm_state_machine_update(void);
void vmc_update_leg_state(void);
void vmc_control_init(void);
void vmc_control_update(void);

#ifdef __cplusplus
}
#endif

#endif
