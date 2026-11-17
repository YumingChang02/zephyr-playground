#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include <stdint.h>
#include <stdbool.h>

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
	char     iso_time[32];
	char     date_str[16];
} telemetry_full_t;

#define DEFAULT_PROF_T_LOW   410
#define DEFAULT_PROF_P_LOW   0
#define DEFAULT_PROF_T_UP    10
#define DEFAULT_PROF_T_HIGH  170
#define DEFAULT_PROF_P_HIGH  90
#define DEFAULT_PROF_T_DROP  10
#define DEFAULT_PROF_TOTAL   (DEFAULT_PROF_T_LOW + DEFAULT_PROF_T_UP + DEFAULT_PROF_T_HIGH + DEFAULT_PROF_T_DROP)

#define APP_MAX_CURRENT_MA   250

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

#endif /* TELEMETRY_H_ */
