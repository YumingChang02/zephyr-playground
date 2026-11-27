#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include <stdint.h>
#include <stdbool.h>

#ifndef STRINGIFY
#define STRINGIFY_HELPER(x) #x
#define STRINGIFY(x)        STRINGIFY_HELPER(x)
#endif

/* Profile Actions */
#define PWM_ACTION_SET_CONFIG_ONLY  0
#define PWM_ACTION_START_PROFILE    1
#define PWM_ACTION_STOP_PROFILE     2
#define PWM_ACTION_MANUAL_PWM       3

/* Profile Stages */
#define PROFILE_STAGE_IDLE          0
#define PROFILE_STAGE_LOW           1
#define PROFILE_STAGE_UP            2
#define PROFILE_STAGE_HIGH          3
#define PROFILE_STAGE_DROP          4

typedef struct __attribute__((packed)) {
	uint32_t sample_id;
	uint32_t motor_rpm;
	int32_t  temp_mc;              /* milli-Celsius (e.g. 42100 = 42.1 C) */
	uint32_t bus_mv;               /* milli-Volts (e.g. 12050 = 12.05 V)  */
	int32_t  current_ma;           /* milli-Amps (signed, e.g. -85 .. 250 mA) */
	int64_t  uptime_ms;
	uint8_t  pwm_pct;              /* current duty cycle 0..100%          */
	uint8_t  profile_stage;        /* PROFILE_STAGE_*                     */
	uint16_t profile_elapsed_sec;  /* which second point is currently at  */
	uint16_t profile_total_sec;    /* total duration of 4 stages in sec   */
	uint8_t  throttle_active;      /* 0=normal, 1=throttling, 2=recovering */
	uint8_t  throttle_drop_pct;    /* active PWM reduction 0..100%        */
	uint16_t throttle_base_rpm;    /* armed baseline RPM (0=unarmed)      */
	int16_t  throttle_base_ma;     /* armed baseline mA                   */
} telemetry_packet_t;

typedef struct {
	uint32_t sample_id;
	uint32_t motor_rpm;
	int32_t  temp_mc;
	uint32_t bus_mv;
	int32_t  current_ma;
	int64_t  uptime_ms;
	uint8_t  pwm_pct;
	uint8_t  profile_stage;
	uint16_t profile_elapsed_sec;
	uint16_t profile_total_sec;
	uint8_t  throttle_active;
	uint8_t  throttle_drop_pct;
	uint16_t throttle_base_rpm;
	int16_t  throttle_base_ma;
	char     iso_time[32];
	char     date_str[16];
} telemetry_full_t;

#ifndef CONFIG_APP_MAX_FG_RPM
#define CONFIG_APP_MAX_FG_RPM 5500
#endif

#ifndef CONFIG_APP_DEFAULT_P_HIGH_PCT
#define CONFIG_APP_DEFAULT_P_HIGH_PCT 80
#endif

#define DEFAULT_PROF_T_LOW   0
#define DEFAULT_PROF_P_LOW   0
#define DEFAULT_PROF_T_UP    0
#define DEFAULT_PROF_T_HIGH  90
#define DEFAULT_PROF_P_HIGH  CONFIG_APP_DEFAULT_P_HIGH_PCT
#define DEFAULT_PROF_T_DROP  0
#define DEFAULT_PROF_TOTAL   (DEFAULT_PROF_T_LOW + DEFAULT_PROF_T_UP + DEFAULT_PROF_T_HIGH + DEFAULT_PROF_T_DROP)

#define APP_MAX_CURRENT_MA   150

typedef struct __attribute__((packed)) {
	uint16_t t_low_sec;
	uint16_t t_up_sec;
	uint16_t t_high_sec;
	uint16_t t_drop_sec;
	uint8_t  p_low_pct;
	uint8_t  p_high_pct;
	uint8_t  action;               /* PWM_ACTION_*                        */
	uint8_t  manual_pwm;           /* 0..100% when action == PWM_ACTION_MANUAL_PWM */
} pwm_profile_ctrl_t;

#define DEFAULT_THROTTLE_ENABLED        1
#define DEFAULT_THROTTLE_MAX_DROP_PCT   60
#define DEFAULT_THROTTLE_SLOPE_PCT_S    5
#define DEFAULT_THROTTLE_RPM_RISE_PCT   10
#define DEFAULT_THROTTLE_CURR_DROP_PCT  15
#define DEFAULT_THROTTLE_COOLDOWN_SEC   8

#define THROTTLE_STATE_NORMAL           0
#define THROTTLE_STATE_ACTIVE           1
#define THROTTLE_STATE_RECOVERING       2

typedef struct __attribute__((packed)) {
	uint8_t  enabled;
	uint8_t  max_drop_pct;
	uint8_t  slope_pct_per_sec;
	uint8_t  rpm_rise_pct;
	uint8_t  curr_drop_pct;
	uint8_t  cooldown_sec;
} pwm_throttle_ctrl_t;

#ifndef CONFIG_APP_CAL_WARMUP_MS
#define CONFIG_APP_CAL_WARMUP_MS 6500
#endif

#ifndef CONFIG_APP_CAL_SETTLE_MS
#define CONFIG_APP_CAL_SETTLE_MS 3500
#endif

#ifndef CONFIG_APP_OP_SETTLE_MS
#define CONFIG_APP_OP_SETTLE_MS 3500
#endif

#define AIR_CAL_MIN_PCT     10
#define AIR_CAL_STEP_PCT    5
#define AIR_CAL_POINTS      19 /* 10%, 15%, 20%, ..., 100% */
#define AIR_CAL_TOTAL_STEPS (AIR_CAL_POINTS + AIR_CAL_POINTS - 1) /* 19 up (10..100) + 18 down (95..10) = 37 steps */

typedef struct {
	uint8_t  pwm_pct;
	uint16_t rpm_up;
	uint32_t power_up;
	uint16_t rpm_down;
	uint32_t power_down;
} air_cal_point_t;

typedef struct {
	bool            valid;
	uint32_t        max_rpm_103_ceil;
	air_cal_point_t pts[AIR_CAL_POINTS];
} air_cal_data_t;

#define DEFAULT_CL_ENABLED          1
#define DEFAULT_CL_TARGET_MAX_PCT   85
#define DEFAULT_CL_LOWER_CLIP_PCT   25
#define DEFAULT_CL_START_PWM_PCT    70
#define DEFAULT_CL_CHECK_UP_SEC     2
#define DEFAULT_CL_CHECK_DOWN_SEC   2
#define DEFAULT_CL_GAIN_UP_PCT      5
#define DEFAULT_CL_GAIN_DOWN_PCT    5
#define DEFAULT_CL_UP_THRESH_PCT    50
#define DEFAULT_CL_DN_THRESH_PCT    25
#define DEFAULT_CL_CAL_RPM_SHIFT    0
#define DEFAULT_CL_CAL_PWR_SHIFT    0
#define DEFAULT_CL_RPM_DROP_PCT     3

typedef struct {
	uint8_t  enabled;          /* 1=closed-loop in HIGH stage, 0=fixed p_high_pct */
	uint8_t  target_max_pct;   /* target max speed as % of max RPM (default 85%) */
	uint8_t  lower_clip_pct;   /* clip PWM to 0% if speed <= this % of max RPM (default 25%) */
	uint8_t  start_pwm_pct;    /* initial test PWM at start of HIGH stage */
	uint8_t  check_up_sec;     /* seconds between speed-up checks */
	uint8_t  check_down_sec;   /* seconds between speed-down checks */
	uint8_t  gain_up_pct;      /* PWM% jump scaling for speed-up */
	uint8_t  gain_down_pct;    /* PWM% jump scaling for speed-down */
	int16_t  up_thresh_pct;    /* % above baseline power (or below RPM) to increase */
	int16_t  dn_thresh_pct;    /* % above baseline power to reduce (hold in between) */
	int8_t   cal_rpm_shift_pct;/* motor degradation RPM shift % */
	int8_t   cal_pwr_shift_pct;/* motor degradation Power shift % */
	uint8_t  rpm_drop_pct;     /* RPM must be this % below air baseline to count as water */
} closed_loop_ctrl_t;

#endif /* TELEMETRY_H_ */
