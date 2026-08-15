#ifndef VMC_PROBE_H
#define VMC_PROBE_H

#include "can.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VMC_WHEEL_LEFT_ID         0x201U
#define VMC_WHEEL_RIGHT_ID        0x202U
#define VMC_WHEEL_CMD_ID          0x200U
#define VMC_WHEEL_CMD_CAN         (&hcan1)
#define VMC_WHEEL_LEFT_CMD_DIRECTION  -1
#define VMC_WHEEL_RIGHT_CMD_DIRECTION  1

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
#define VMC_LEG_LENGTH_KP_DEFAULT  1000.0f
#define VMC_LEG_LENGTH_KD_DEFAULT  100.0f
#define VMC_LEG_FORCE_MAX_DEFAULT  120.0f
#define VMC_LEG_LENGTH_PID_DIRECTION -1.0f
#define VMC_LEG_PHI0_TARGET_DEFAULT VMC_PHI0_VERTICAL_RAD
#define VMC_LEG_PHI0_KP_DEFAULT     50.0f
#define VMC_LEG_PHI0_KD_DEFAULT     0.5f
#define VMC_LEG_PHI0_TORQUE_MAX_DEFAULT 100.0f
#define VMC_LEG_PHI0_PID_DIRECTION  1.0f
#define VMC_LEG_SUPPORT_FORCE_DEFAULT 0.0f
#define VMC_LEG_SUPPORT_FORCE_MAX     120.0f
#define VMC_LEG_SUPPORT_FORCE_DIRECTION -1.0f
#define VMC_LEG_TARGET_SHORT      0.180f
#define VMC_LEG_TARGET_MID        0.280f
#define VMC_LEG_TARGET_LONG       0.380f
#define VMC_LEG_LENGTH_RAMP_RATE_DEFAULT 0.200f
#define VMC_CONTROL_DT            0.001f
#define VMC_FEEDBACK_TIMEOUT_MS   100U
#define VMC_WHEEL_RADIUS          0.065f
#define VMC_PHI0_VERTICAL_RAD     1.5707963267948966f
#define VMC_BODY_PITCH_DIRECTION  1.0f
#define VMC_BODY_PITCH_RATE_DIRECTION 1.0f
#define VMC_BODY_MOTION_X_LIMIT_DEFAULT 100.0f

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

#define VMC_FEEDBACK_TIMEOUT_NONE        0x00U
#define VMC_FEEDBACK_TIMEOUT_RIGHT_BACK  0x01U
#define VMC_FEEDBACK_TIMEOUT_RIGHT_AHEAD 0x02U
#define VMC_FEEDBACK_TIMEOUT_LEFT_AHEAD  0x04U
#define VMC_FEEDBACK_TIMEOUT_LEFT_BACK   0x08U
#define VMC_FEEDBACK_TIMEOUT_WHEEL_LEFT  0x10U
#define VMC_FEEDBACK_TIMEOUT_WHEEL_RIGHT 0x20U

#define VMC_WHEEL_CURRENT_MAX_DEFAULT 16000
#define VMC_BALANCE_WHEEL_CURRENT_LIMIT_DEFAULT 5000
#define VMC_BALANCE_WHEEL_TORQUE_LIMIT_DEFAULT 3.3f
#define VMC_BALANCE_WHEEL_GEAR_RATIO 15.7647f
#define VMC_BALANCE_WHEEL_TORQUE_CONSTANT 0.3f
#define VMC_BALANCE_C620_MAX_CURRENT_A 20.0f
#define VMC_BALANCE_C620_RAW_MAX 16384.0f
#define VMC_BALANCE_WHEEL_TORQUE_TO_CURRENT \
  (VMC_BALANCE_C620_RAW_MAX / \
   (VMC_BALANCE_WHEEL_GEAR_RATIO * VMC_BALANCE_WHEEL_TORQUE_CONSTANT * \
    VMC_BALANCE_C620_MAX_CURRENT_A))
#define VMC_BALANCE_WHEEL_CURRENT_GAIN_DEFAULT 10.0f
#define VMC_BALANCE_TP_LIMIT_DEFAULT 5.0f
#define VMC_BALANCE_PITCH_OFFSET_DEFAULT 0.0f
#define VMC_BALANCE_PHI_OFFSET_DEFAULT -0.04f
#define VMC_BALANCE_DX_FEEDBACK_ENABLE_DEFAULT 1U
#define VMC_BALANCE_THETA_DEADBAND_DEFAULT 0.0f // 死区
#define VMC_BALANCE_PHI_DEADBAND_DEFAULT 0.0f // 死区 0.010f
#define VMC_BALANCE_X_FEEDBACK_ENABLE_DEFAULT 1U
#define VMC_BALANCE_X_FEEDBACK_GAIN_DEFAULT -1.0f
#define VMC_BALANCE_DX_FEEDBACK_GAIN_DEFAULT -1.0f
#define VMC_BALANCE_X_FEEDBACK_LIMIT_DEFAULT 0.5f
#define VMC_BALANCE_DX_FEEDBACK_LIMIT_DEFAULT 1.5f
#define VMC_BALANCE_SPEED_CONTROL_ENABLE_DEFAULT 1U
#define VMC_BALANCE_SPEED_TARGET_MAX_DEFAULT 0.8f
#define VMC_BALANCE_SPEED_RAMP_RATE_DEFAULT 1.5f
#define VMC_BALANCE_SPEED_DEADBAND_DEFAULT 0.02f
#define VMC_BALANCE_STOP_SPEED_THRESHOLD_DEFAULT 0.08f
#define VMC_BALANCE_SPEED_PITCH_GAIN_DEFAULT 0.0f
#define VMC_BALANCE_SPEED_PITCH_LIMIT_DEFAULT 0.12f
#define VMC_BALANCE_SPEED_PITCH_RAMP_RATE_DEFAULT 0.5f
#define VMC_BALANCE_DTHETA_LPF_ALPHA_DEFAULT 0.1f
#define VMC_RC_CH_DEADBAND 20
#define VMC_RC_WHEEL_FWD_CURRENT_MAX 2000
#define VMC_RC_WHEEL_TURN_CURRENT_MAX 1200
#define VMC_YAW_HOLD_ENABLE_DEFAULT 1U
#define VMC_YAW_HOLD_KP_DEFAULT 500.0f
#define VMC_YAW_HOLD_KD_DEFAULT 60.0f
#define VMC_YAW_HOLD_DEADBAND_DEFAULT 0.01f
#define VMC_YAW_HOLD_CURRENT_LIMIT_DEFAULT 800
#define VMC_YAW_HOLD_DIRECTION 1.0f

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
  double phi0_error;
  double phi0_p_output;
  double phi0_d_output;
  double phi0_pid_output;
  double support_force_output;
  double force_command;
  double torque_phi0_command;
  double joint_torque_ahead;
  double joint_torque_back;
} vmc_leg_state_t;

typedef struct {
  uint8_t enabled;
  float target_length;
  float ramped_target_length;
  float length_ramp_rate;
  float kp_length;
  float kd_length;
  float target_phi0;
  float kp_phi0;
  float kd_phi0;
  float max_phi0_torque;
  float max_force;
  float support_force;
  uint8_t ramp_active;
} vmc_control_t;

typedef struct {
  uint8_t enabled;
  int16_t left_current;
  int16_t right_current;
  int16_t max_current;
  int16_t manual_left_current;
  int16_t manual_right_current;
  int16_t manual_forward_current;
  int16_t manual_turn_current;
  int16_t yaw_hold_current;
  int16_t left_current_output;
  int16_t right_current_output;
} vmc_wheel_control_t;

typedef struct {
  uint8_t enabled;
  uint8_t wheel_output_enable;
  uint8_t tp_output_enable;
  uint8_t dx_feedback_enable;
  uint8_t x_feedback_enable;
  uint8_t safety_allows_output;
  uint8_t yaw_hold_enable;
  uint8_t yaw_hold_active;
  float pitch_offset;
  float phi_offset;
  float theta_deadband;
  float phi_deadband;
  float x_offset;
  float x_limit;
  float x_feedback_gain;
  float dx_feedback_gain;
  float x_feedback_limit;
  float dx_feedback_limit;
  uint8_t speed_control_enable;
  float speed_target;
  float speed_target_ramped;
  float speed_target_max;
  float speed_ramp_rate;
  float speed_deadband;
  float stop_speed_threshold;
  float speed_pitch_gain;
  float speed_pitch_limit;
  float speed_pitch_ramp_rate;
  float speed_pitch_offset;
  uint8_t speed_stop_hold_active;
  float dtheta_lpf_alpha;
  float yaw_target;
  float yaw_hold_kp;
  float yaw_hold_kd;
  float yaw_hold_deadband;
  int16_t yaw_hold_current_limit;
  int16_t wheel_current_limit;
  float wheel_torque_limit;
  float wheel_torque_to_current;
  float wheel_current_gain;
  float tp_limit;
  double lqr_theta;
  double lqr_d_theta;
  double lqr_d_theta_raw;
  double theta_error_raw;
  double theta_error;
  double phi_error_raw;
  double phi_error;
  double x_error;
  double dx_error;
  double dx_target;
  double dx_raw;
  double yaw_error_raw;
  double yaw_error;
  double yaw_rate_error;
  double left_wheel_theta_term;
  double left_wheel_dtheta_term;
  double left_wheel_x_term;
  double left_wheel_dx_term;
  double left_wheel_phi_term;
  double left_wheel_dphi_term;
  double right_wheel_theta_term;
  double right_wheel_dtheta_term;
  double right_wheel_x_term;
  double right_wheel_dx_term;
  double right_wheel_phi_term;
  double right_wheel_dphi_term;
  double wheel_torque_raw;
  double left_wheel_torque_raw;
  double right_wheel_torque_raw;
  double left_wheel_torque_limited;
  double right_wheel_torque_limited;
  double left_tp_raw;
  double right_tp_raw;
  int16_t left_wheel_current_cmd;
  int16_t right_wheel_current_cmd;
  int16_t left_wheel_current_send;
  int16_t right_wheel_current_send;
  float left_tp_cmd;
  float right_tp_cmd;
} vmc_balance_t;

typedef struct {
  float roll;
  float pitch;
  float yaw;
  float pitch_rate;
  float wx;
  float wy;
  float wz;
  float ax;
  float ay;
  float az;
  uint8_t imu_connected;
  uint8_t imu_valid;
} vmc_body_state_t;

typedef struct {
  double left_theta;
  double left_d_theta;
  double right_theta;
  double right_d_theta;
  double theta;
  double d_theta;
  double phi;
  double d_phi;
  double left_wheel_w;
  double right_wheel_w;
  double wheel_dx;
  double wheel_dx_gap;
  double left_body_dx;
  double right_body_dx;
  double d_x_raw;
  double dd_x;
  double kf_x;
  double kf_dx;
  double kf_acc;
  double leg_length_avg;
} vmc_motion_state_t;

typedef struct {
  double left_k[12];
  double right_k[12];
  double rtt_k[12];
  uint8_t rtt_override_active;
  uint8_t rtt_last_error;
  uint32_t rtt_update_count;
  uint32_t rtt_error_count;
  uint32_t rtt_last_update_ms;
  uint32_t rtt_rx_count;
} vmc_lqr_state_t;

typedef struct {
  vmc_wheel_motor_t wheel_left;
  vmc_wheel_motor_t wheel_right;
  vmc_joint_motor_t joint_right_back;
  vmc_joint_motor_t joint_right_ahead;
  vmc_joint_motor_t joint_left_ahead;
  vmc_joint_motor_t joint_left_back;
  vmc_leg_state_t left_leg;
  vmc_leg_state_t right_leg;
  vmc_body_state_t body;
  vmc_motion_state_t motion;
  vmc_lqr_state_t lqr;
  vmc_balance_t balance;
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
  uint32_t wheel_tx_count;
  uint32_t wheel_tx_fail_count;
  uint8_t dm_state;
  uint8_t dm_tx_phase;
  uint8_t phi0_protect_latched;
  uint8_t phi0_protect_source;
  uint8_t feedback_timeout_active;
  uint8_t feedback_timeout_source;
} vmc_probe_state_t;

extern volatile vmc_probe_state_t vmc_probe_state;
extern volatile vmc_control_t vmc_control;
extern volatile vmc_wheel_control_t vmc_wheel_control;
extern volatile vmc_balance_t vmc_balance;

void vmc_probe_task(void const *argument);
void vmc_can_init(void);
void vmc_can_poll_joints_zero(void);
void vmc_dm_state_machine_update(void);
void vmc_update_leg_state(void);
void vmc_control_init(void);
void vmc_control_update(void);
void vmc_wheel_control_init(void);
void vmc_wheel_control_update(void);
void vmc_balance_init(void);
void vmc_balance_update(void);

#ifdef __cplusplus
}
#endif

#endif
