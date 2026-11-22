#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include <stdint.h>
#include <stdbool.h>

#ifndef STRINGIFY
#define STRINGIFY_HELPER(x) #x
#define STRINGIFY(x)        STRINGIFY_HELPER(x)
#endif

#define IPM_CHANNEL_APP_TELEMETRY   2   /* ESP32 IPM Channels 0 & 1 reserved for BT/WiFi */
#define IPM_CHANNEL_APP_PRINT       3   /* IPC printk channel from APP_CPU to PRO_CPU */
#define IPM_CHANNEL_TIME_SYNC       4   /* IPC time sync channel from PRO_CPU to APP_CPU */
#define IPM_CHANNEL_PWM_CTRL        5   /* IPC target PWM & 4-stage profile command */
#define IPM_CHANNEL_THROTTLE_CTRL   6   /* IPC dry-run PWM throttle config */

#define IPC_PRINT_MAX_LEN           192

/* Actions for IPM_CHANNEL_PWM_CTRL */
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

/* Full telemetry record with wall-clock time added by PRO_CPU */
typedef struct {
	uint32_t sample_id;
	uint32_t motor_rpm;
	int32_t  temp_mc;              /* milli-Celsius (e.g. 42100 = 42.1 C) */
	uint32_t bus_mv;               /* milli-Volts (e.g. 12050 = 12.05 V)  */
	int32_t  current_ma;           /* milli-Amps (signed, e.g. -85 .. 250 mA) */
	int64_t  uptime_ms;
	uint8_t  pwm_pct;
	uint8_t  profile_stage;
	uint16_t profile_elapsed_sec;
	uint16_t profile_total_sec;
	uint8_t  throttle_active;
	uint8_t  throttle_drop_pct;
	uint16_t throttle_base_rpm;
	int16_t  throttle_base_ma;
	char     iso_time[32];         /* "YYYY-MM-DDTHH:MM:SSZ"              */
	char     date_str[16];         /* "YYYY-MM-DD"                        */
} telemetry_full_t;

typedef struct __attribute__((packed)) {
	char msg[IPC_PRINT_MAX_LEN];
} ipc_print_packet_t;

typedef struct __attribute__((packed)) {
	uint8_t  month;
	uint8_t  day;
	uint8_t  hour;
	uint8_t  min;
	uint8_t  sec;
	uint8_t  synced;
	uint16_t ms_part;
	uint32_t sec_of_day;
} ipm_time_sync_t;

#ifndef CONFIG_APP_MAX_FG_RPM
#define CONFIG_APP_MAX_FG_RPM 5500
#endif

#define DEFAULT_PROF_T_LOW   410
#define DEFAULT_PROF_P_LOW   0
#define DEFAULT_PROF_T_UP    10
#define DEFAULT_PROF_T_HIGH  170
#define DEFAULT_PROF_P_HIGH  90
#define DEFAULT_PROF_T_DROP  10
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
} ipm_pwm_ctrl_t;

typedef ipm_pwm_ctrl_t pwm_profile_ctrl_t;

#define DEFAULT_THROTTLE_ENABLED        0
#define DEFAULT_THROTTLE_MAX_DROP_PCT   30
#define DEFAULT_THROTTLE_SLOPE_PCT_S    5
#define DEFAULT_THROTTLE_RPM_RISE_PCT   15
#define DEFAULT_THROTTLE_CURR_DROP_PCT  20
#define DEFAULT_THROTTLE_COOLDOWN_SEC   8

#define THROTTLE_STATE_NORMAL           0
#define THROTTLE_STATE_ACTIVE           1
#define THROTTLE_STATE_RECOVERING       2

typedef struct __attribute__((packed)) {
	uint8_t  enabled;              /* 0 = disabled, 1 = enabled */
	uint8_t  max_drop_pct;         /* X%: max PWM reduction (0..100%) */
	uint8_t  slope_pct_per_sec;    /* Y%/s: ramp rate (1..100%/s) */
	uint8_t  rpm_rise_pct;         /* RPM rise % vs baseline (1..200%) */
	uint8_t  curr_drop_pct;        /* Current drop % vs baseline (1..100%) */
	uint8_t  cooldown_sec;         /* Hold duration before recovery probe (1..120s) */
} ipm_throttle_ctrl_t;

#endif /* TELEMETRY_H_ */
