/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Waveshare ESP32-S3-GEEK APP_CPU (Core 1): Real-Time Sensor & Motor Control Engine.
 *
 * Architecture:
 *   [HW Timer ISR (1~10 Hz)] -> signals sem -> [Thread 1: Speed & PWM (Priority 3)]
 *                                                   │ (motor_speed_msgq)
 *                                                   ▼
 *                                              [Thread 2: Sensor & IPM (Priority 5)]
 *                                                   ├── (ipm_send channel 2: telemetry)
 *                                                   └── (ipm_send channel 3: printk redirect)
 *                                                   ▼
 *                                              [PRO_CPU Core 0 (Wi-Fi, SD, Web, Console)]
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/drivers/display.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "telemetry.h"

static const struct device *ipm_dev;

static void app_printk(const char *fmt, ...)
{
	if (!ipm_dev) {
		ipm_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(ipm0));
	}
	if (!ipm_dev || !device_is_ready(ipm_dev)) {
		return;
	}

	ipc_print_packet_t pkt;
	va_list ap;
	va_start(ap, fmt);
	int len = vsnprintf(pkt.msg, sizeof(pkt.msg), fmt, ap);
	va_end(ap);

	if (len > 0) {
		size_t send_len = (size_t)len + 1;
		if (send_len > sizeof(pkt.msg)) {
			send_len = sizeof(pkt.msg);
			pkt.msg[sizeof(pkt.msg) - 1] = '\0';
		}
		ipm_send(ipm_dev, 0, IPM_CHANNEL_APP_PRINT, pkt.msg, (int)send_len);
	}
}

#undef printk
#define printk(fmt, ...) app_printk(fmt, ##__VA_ARGS__)

#define CONFIG_APP_SAMPLE_RATE_HZ       3     /* Configured to 3 Hz */
#ifndef CONFIG_APP_MOTOR_PULSES_PER_REV
#define CONFIG_APP_MOTOR_PULSES_PER_REV 6     /* 6 pulses per revolution */
#endif
#define TIMER_INTERVAL_US               (1000000 / CONFIG_APP_SAMPLE_RATE_HZ)

/* Configurable Upper Scales for Display Gauge Bars */
#ifndef CONFIG_APP_MAX_FG_RPM
#define CONFIG_APP_MAX_FG_RPM           5500    /* 100% scale for Motor FG RPM */
#endif

#ifndef CONFIG_APP_MAX_POWER_W
#define CONFIG_APP_MAX_POWER_W          25      /* 100% scale for Power (Watts) */
#endif

#define PRIORITY_HIGH                   3     /* Thread 1: Motor Speed Sampler (FG Counter) */
#define PRIORITY_MEDIUM                 5     /* Thread 2: Sensor Aggregator (I2C/INA219)   */
#define PRIORITY_DISPLAY                9     /* Thread 3: Low-Priority Display Engine */
#define THREAD_STACK_SIZE               2048
#define DISPLAY_STACK_SIZE              3072

#define TIMER_NODE                      DT_ALIAS(app_timer)

BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(TIMER_NODE),
             "Devicetree error: 'app-timer' alias is missing or disabled!");

/* Hardware Devicetree Specs for Sensor Acquisition */
static const struct gpio_dt_spec fg_spec = GPIO_DT_SPEC_GET_OR(DT_ALIAS(fg_pin), gpios, {0});
static const struct pwm_dt_spec pwm_motor = PWM_DT_SPEC_GET_OR(DT_ALIAS(motor_pwm), {0});
static const struct device *const ina219_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(ina219));
static const struct device *const display_dev = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_display));
static const struct gpio_dt_spec bl_spec = GPIO_DT_SPEC_GET_OR(DT_ALIAS(lcd_bl), gpios, {0});

/* Gauge values shared with Thread 3 */
static volatile uint32_t g_disp_pwm_pct = 0;
static volatile uint32_t g_disp_rpm = 0;
static volatile int32_t  g_disp_current_ma = 0;

/* Real-Time Clock values synced from PRO_CPU */
static volatile uint8_t  g_time_month = 1;
static volatile uint8_t  g_time_day = 1;
static volatile uint8_t  g_time_hour = 0;
static volatile uint8_t  g_time_min = 0;
static volatile uint8_t  g_time_sec = 0;
static volatile bool     g_time_synced = false;
static volatile uint16_t g_time_ms_part = 0;
static volatile uint32_t g_time_sec_of_day = 0;
static volatile int64_t  g_time_sync_uptime_ms = 0;

/* Remote PWM Target / 4-Stage Profile Configuration from PRO_CPU */
static volatile ipm_pwm_ctrl_t g_prof_cfg = {
	.t_low_sec = DEFAULT_PROF_T_LOW,
	.t_up_sec = DEFAULT_PROF_T_UP,
	.t_high_sec = DEFAULT_PROF_T_HIGH,
	.t_drop_sec = DEFAULT_PROF_T_DROP,
	.p_low_pct = DEFAULT_PROF_P_LOW,
	.p_high_pct = DEFAULT_PROF_P_HIGH,
	.action = PWM_ACTION_START_PROFILE,
	.manual_pwm = 0,
};

static volatile uint8_t  g_prof_stage = PROFILE_STAGE_LOW;
static volatile uint16_t g_prof_elapsed_sec = 0;
static volatile uint16_t g_prof_total_sec = DEFAULT_PROF_TOTAL;
static volatile uint32_t g_active_pwm_pct = 0;
static volatile bool     g_prof_log_pending = false;

static void app_ipm_callback(const struct device *dev, void *user_data,
                             uint32_t id, volatile void *data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (id == IPM_CHANNEL_TIME_SYNC && data != NULL) {
		const ipm_time_sync_t *ts = (const ipm_time_sync_t *)data;
		g_time_month = ts->month;
		g_time_day = ts->day;
		g_time_hour = ts->hour;
		g_time_min = ts->min;
		g_time_sec = ts->sec;
		g_time_synced = (ts->synced != 0);
		g_time_ms_part = ts->ms_part;
		g_time_sec_of_day = ts->sec_of_day;
		g_time_sync_uptime_ms = k_uptime_get();
	} else if (id == IPM_CHANNEL_PWM_CTRL && data != NULL) {
		const ipm_pwm_ctrl_t *ctrl = (const ipm_pwm_ctrl_t *)data;
		if (ctrl->action == PWM_ACTION_SET_CONFIG_ONLY) {
			/* Live on-the-fly curve reconfiguration while running or idle */
			g_prof_cfg.t_low_sec = ctrl->t_low_sec;
			g_prof_cfg.t_up_sec = ctrl->t_up_sec;
			g_prof_cfg.t_high_sec = ctrl->t_high_sec;
			g_prof_cfg.t_drop_sec = ctrl->t_drop_sec;
			g_prof_cfg.p_low_pct = ctrl->p_low_pct;
			g_prof_cfg.p_high_pct = ctrl->p_high_pct;
			g_prof_total_sec = ctrl->t_low_sec + ctrl->t_up_sec + ctrl->t_high_sec + ctrl->t_drop_sec;
			g_prof_log_pending = true;
		} else if (ctrl->action == PWM_ACTION_START_PROFILE) {
			/* Start / Resume continuous profile loop */
			g_prof_cfg = *ctrl;
			g_prof_total_sec = ctrl->t_low_sec + ctrl->t_up_sec + ctrl->t_high_sec + ctrl->t_drop_sec;
			g_prof_stage = PROFILE_STAGE_LOW;
			g_prof_log_pending = true;
		} else if (ctrl->action == PWM_ACTION_STOP_PROFILE) {
			g_prof_stage = PROFILE_STAGE_IDLE;
			g_prof_elapsed_sec = 0;
			g_active_pwm_pct = 0;
			if (pwm_motor.dev && device_is_ready(pwm_motor.dev)) {
				pwm_set_dt(&pwm_motor, pwm_motor.period, 0);
			}
			g_disp_pwm_pct = 0;
			g_prof_log_pending = true;
		} else if (ctrl->action == PWM_ACTION_MANUAL_PWM) {
			g_prof_stage = PROFILE_STAGE_IDLE;
			g_prof_elapsed_sec = 0;
			uint32_t val = ctrl->manual_pwm;
			if (val > 100) val = 100;
			g_active_pwm_pct = val;
			if (pwm_motor.dev && device_is_ready(pwm_motor.dev)) {
				uint32_t pulse_ns = (pwm_motor.period * val) / 100;
				pwm_set_dt(&pwm_motor, pwm_motor.period, pulse_ns);
			}
			g_disp_pwm_pct = val;
			g_prof_log_pending = true;
		}
	}
}

static volatile uint32_t g_fg_pulse_counter = 0;
static uint64_t g_total_pulses = 0;
static struct gpio_callback fg_gpio_cb;

static void fg_gpio_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	g_fg_pulse_counter++;
}

typedef struct {
	uint32_t pulse_count;
	uint32_t rpm;
	int64_t  timestamp_ms;
} motor_speed_sample_t;

K_MSGQ_DEFINE(motor_speed_msgq, sizeof(motor_speed_sample_t), 4, 4);
static K_SEM_DEFINE(timer_tick_sem, 0, 1);

static uint32_t prng_state = 123456789;
static uint32_t prng_range(uint32_t min, uint32_t max)
{
	prng_state = prng_state * 1664525 + 1013904223;
	return min + (prng_state % (max - min + 1));
}

static void timer_top_subroutine(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	k_sem_give(&timer_tick_sem);
}

/* ------------------------------------------------------------------------- */
/* Thread 1: Motor Speed & PWM Engine                                        */
/* ------------------------------------------------------------------------- */
static void motor_speed_thread_fn(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	int64_t last_time_ms = k_uptime_get();
	uint32_t duty_cycle_pct = 0;
	g_disp_pwm_pct = duty_cycle_pct;

	if (fg_spec.port && device_is_ready(fg_spec.port)) {
		int cfg_ret = gpio_pin_configure_dt(&fg_spec, GPIO_INPUT);
		gpio_init_callback(&fg_gpio_cb, fg_gpio_isr, BIT(fg_spec.pin));
		gpio_add_callback(fg_spec.port, &fg_gpio_cb);
		int int_ret = gpio_pin_interrupt_configure_dt(&fg_spec, GPIO_INT_EDGE_FALLING);
		int initial_level = gpio_pin_get_dt(&fg_spec);
		printk("[APP_CPU] Real GPIO FG Counter initialized on pin %d (cfg=%d, int=%d, initial level=%d)\n",
		       fg_spec.pin, cfg_ret, int_ret, initial_level);
	} else {
		printk("[APP_CPU] FG sensor GPIO not ready, using simulated pulses\n");
	}

	if (pwm_motor.dev && device_is_ready(pwm_motor.dev)) {
		uint32_t pulse_ns = (pwm_motor.period * duty_cycle_pct) / 100;
		int ret = pwm_set_dt(&pwm_motor, pwm_motor.period, pulse_ns);
		printk("[APP_CPU] Hardware PWM initialized at %llu ns period (%d%% duty, ret=%d)\n",
		       (unsigned long long)pwm_motor.period, duty_cycle_pct, ret);
	}

	uint32_t sample_loop_cnt = 0;

	while (1) {
		k_sem_take(&timer_tick_sem, K_FOREVER);
		sample_loop_cnt++;
		int64_t now_ms = k_uptime_get();
		int64_t elapsed_ms = now_ms - last_time_ms;
		if (elapsed_ms <= 0) {
			elapsed_ms = 1;
		}

		if (g_prof_log_pending) {
			g_prof_log_pending = false;
			printk("[APP_CPU] Profile Action: stage=%u, low=%u%% (%us), up=(%us), high=%u%% (%us), drop=(%us), total=%us\n",
			       g_prof_stage, g_prof_cfg.p_low_pct, g_prof_cfg.t_low_sec, g_prof_cfg.t_up_sec,
			       g_prof_cfg.p_high_pct, g_prof_cfg.t_high_sec, g_prof_cfg.t_drop_sec, g_prof_total_sec);
		}

		/* Evaluate autonomous looping 4-stage profile curve aligned to 00:00:00 */
		if (g_prof_stage != PROFILE_STAGE_IDLE) {
			uint32_t t1 = g_prof_cfg.t_low_sec;
			uint32_t t2 = g_prof_cfg.t_up_sec;
			uint32_t t3 = g_prof_cfg.t_high_sec;
			uint32_t t4 = g_prof_cfg.t_drop_sec;
			uint32_t pL = g_prof_cfg.p_low_pct;
			uint32_t pH = g_prof_cfg.p_high_pct;
			uint32_t total = t1 + t2 + t3 + t4;
			g_prof_total_sec = (uint16_t)total;

			uint32_t target_duty = pL;

			if (total == 0) {
				g_prof_elapsed_sec = 0;
				target_duty = pL;
			} else {
				/* Compute wall-clock aligned time relative to 00:00:00 if synced, else uptime */
				uint32_t cur_sec_of_day;
				uint32_t cur_sub_ms;
				if (g_time_synced) {
					int64_t ms_since_sync = now_ms - g_time_sync_uptime_ms;
					if (ms_since_sync < 0) {
						ms_since_sync = 0;
					}
					uint64_t total_day_ms = (uint64_t)g_time_sec_of_day * 1000ULL +
					                        (uint64_t)g_time_ms_part +
					                        (uint64_t)ms_since_sync;
					cur_sec_of_day = (uint32_t)((total_day_ms / 1000ULL) % 86400ULL);
					cur_sub_ms = (uint32_t)(total_day_ms % 1000ULL);
				} else {
					cur_sec_of_day = (uint32_t)(now_ms / 1000ULL);
					cur_sub_ms = (uint32_t)(now_ms % 1000ULL);
				}

				/* Loop position strictly aligned to 00:00:00 */
				uint32_t elapsed_sec = cur_sec_of_day % total;
				g_prof_elapsed_sec = (uint16_t)elapsed_sec;

				/* Evaluate 4-stage profile: Low -> Ramp Up -> High -> Drop */
				if (elapsed_sec < t1) {
					g_prof_stage = PROFILE_STAGE_LOW;
					target_duty = pL;
				} else if (elapsed_sec < t1 + t2) {
					g_prof_stage = PROFILE_STAGE_UP;
					uint32_t ramp_total_ms = t2 * 1000U;
					uint32_t dt_ms = (elapsed_sec - t1) * 1000U + cur_sub_ms;
					if (dt_ms > ramp_total_ms) {
						dt_ms = ramp_total_ms;
					}
					if (pH >= pL) {
						target_duty = pL + (ramp_total_ms > 0 ? ((pH - pL) * dt_ms) / ramp_total_ms : (pH - pL));
					} else {
						target_duty = pL - (ramp_total_ms > 0 ? ((pL - pH) * dt_ms) / ramp_total_ms : (pL - pH));
					}
				} else if (elapsed_sec < t1 + t2 + t3) {
					g_prof_stage = PROFILE_STAGE_HIGH;
					target_duty = pH;
				} else {
					g_prof_stage = PROFILE_STAGE_DROP;
					uint32_t drop_total_ms = t4 * 1000U;
					uint32_t dt_ms = (elapsed_sec - (t1 + t2 + t3)) * 1000U + cur_sub_ms;
					if (dt_ms > drop_total_ms) {
						dt_ms = drop_total_ms;
					}
					if (pH >= pL) {
						target_duty = pH - (drop_total_ms > 0 ? ((pH - pL) * dt_ms) / drop_total_ms : (pH - pL));
					} else {
						target_duty = pH + (drop_total_ms > 0 ? ((pL - pH) * dt_ms) / drop_total_ms : (pL - pH));
					}
				}
			}

			if (target_duty > 100) {
				target_duty = 100;
			}
			if (target_duty != g_active_pwm_pct) {
				g_active_pwm_pct = target_duty;
				if (pwm_motor.dev && device_is_ready(pwm_motor.dev)) {
					uint32_t pulse_ns = (pwm_motor.period * target_duty) / 100;
					pwm_set_dt(&pwm_motor, pwm_motor.period, pulse_ns);
				}
				g_disp_pwm_pct = target_duty;
			}
		}
		duty_cycle_pct = g_active_pwm_pct;

		uint32_t delta_pulses;
		if (fg_spec.port && device_is_ready(fg_spec.port)) {
			unsigned int key = irq_lock();
			delta_pulses = g_fg_pulse_counter;
			g_fg_pulse_counter = 0; /* Reset on read: prevents overflow and eliminates wrapping issues! */
			irq_unlock(key);
		} else {
			/* Simulation: ~3000 RPM equivalent */
			delta_pulses = prng_range(8, 12);
		}
		g_total_pulses += delta_pulses;
		last_time_ms = now_ms;

		uint32_t raw_rpm = (uint32_t)(((uint64_t)delta_pulses * 60000ULL) /
		                              (CONFIG_APP_MOTOR_PULSES_PER_REV * (uint64_t)elapsed_ms));

		/* Exponential Moving Average (EMA) to smooth out pulse quantization jitter */
		static uint32_t s_filtered_rpm = 0;
		if (delta_pulses == 0) {
			s_filtered_rpm = (s_filtered_rpm * 6) / 10;
		} else if (s_filtered_rpm == 0) {
			s_filtered_rpm = raw_rpm;
		} else {
			s_filtered_rpm = (s_filtered_rpm * 65 + raw_rpm * 35) / 100;
		}
		uint32_t rpm = s_filtered_rpm;

		if ((sample_loop_cnt % 6) == 0 && fg_spec.port && device_is_ready(fg_spec.port)) {
			int current_level = gpio_pin_get_dt(&fg_spec);
			printk("[APP_CPU] Motor: PWM %u%% | Pulses: %u in %u ms (total: %llu) | Raw: %u | Smoothed RPM: %u | FG line: %d\n",
			       duty_cycle_pct, delta_pulses, (uint32_t)elapsed_ms, (unsigned long long)g_total_pulses, raw_rpm, rpm, current_level);
		}

		motor_speed_sample_t sample = {
			.pulse_count = delta_pulses,
			.rpm = rpm,
			.timestamp_ms = now_ms,
		};

		g_disp_rpm = rpm;

		k_msgq_put(&motor_speed_msgq, &sample, K_NO_WAIT);
	}
}

/* ------------------------------------------------------------------------- */
/* Thread 2: Sensor Aggregator & IPM Sender                                  */
/* ------------------------------------------------------------------------- */
static void sensor_aggregator_thread_fn(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	motor_speed_sample_t speed_sample;
	uint32_t sample_id = 0;
	bool has_ina219 = (ina219_dev && device_is_ready(ina219_dev));

	if (has_ina219) {
		printk("[APP_CPU] Real INA219 sensor detected on I2C\n");
	} else {
		printk("[APP_CPU] INA219 not detected, using simulation values\n");
	}

	while (1) {
		k_msgq_get(&motor_speed_msgq, &speed_sample, K_FOREVER);
		sample_id++;

		uint32_t bus_mv = 0;
		int32_t current_ma = 0;
		int32_t temp_mc = 40000 + (int32_t)prng_range(0, 3000); /* 40.0 ~ 43.0 C */

		if (has_ina219) {
			struct sensor_value v_bus, i_shunt;
			if (sensor_sample_fetch(ina219_dev) == 0 &&
			    sensor_channel_get(ina219_dev, SENSOR_CHAN_VOLTAGE, &v_bus) == 0 &&
			    sensor_channel_get(ina219_dev, SENSOR_CHAN_CURRENT, &i_shunt) == 0) {
				bus_mv = (uint32_t)(v_bus.val1 * 1000 + v_bus.val2 / 1000);
				current_ma = (int32_t)(i_shunt.val1 * 1000 + i_shunt.val2 / 1000);
			} else {
				bus_mv = 12000 + prng_range(0, 100);
				current_ma = 150 + (int32_t)prng_range(0, 20);
			}
		} else {
			bus_mv = 12000 + prng_range(0, 100);
			current_ma = 100 + (int32_t)((speed_sample.rpm * 120) / CONFIG_APP_MAX_FG_RPM) + (int32_t)prng_range(0, 10);
		}

		g_disp_current_ma = current_ma;

		telemetry_packet_t pkt = {
			.sample_id = sample_id,
			.motor_rpm = speed_sample.rpm,
			.temp_mc = temp_mc,
			.bus_mv = bus_mv,
			.current_ma = current_ma,
			.uptime_ms = speed_sample.timestamp_ms,
			.pwm_pct = (uint8_t)g_active_pwm_pct,
			.profile_stage = (uint8_t)g_prof_stage,
			.profile_elapsed_sec = (uint16_t)g_prof_elapsed_sec,
			.profile_total_sec = (uint16_t)g_prof_total_sec,
		};

		if (ipm_dev) {
			int ret = ipm_send(ipm_dev, 0, IPM_CHANNEL_APP_TELEMETRY, &pkt, sizeof(pkt));
			if (ret != 0 && (sample_id % 50 == 0)) {
				printk("[APP_CPU] IPM send error: %d\n", ret);
			}
		}
	}
}

/* ------------------------------------------------------------------------- */
/* Thread 3: Low-Priority Display Engine (Stacked Bars + Right Time Widget)  */
/* ------------------------------------------------------------------------- */
#define GAUGE_BAR_X     4
#define GAUGE_BAR_W     158
#define GAUGE_BAR_H     44

#define TIME_PANEL_X    166
#define TIME_PANEL_Y    2
#define TIME_PANEL_W    70
#define TIME_PANEL_H    132

#define DRAW_BUF_SIZE   (TIME_PANEL_W * TIME_PANEL_H) /* 9240 uint16_t = 18.4 KB */
static uint16_t g_draw_buf[DRAW_BUF_SIZE];

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
	uint16_t c = ((r & 0x1F) << 11) | ((g & 0x3F) << 5) | (b & 0x1F);
	return sys_cpu_to_be16(c);
}

/* 6x11 Monochrome Bitmap Font */
static const uint8_t font_6x11[24][11] = {
	/* 0 */ { 0x1E, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x1E },
	/* 1 */ { 0x0C, 0x1C, 0x3C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x3F },
	/* 2 */ { 0x1E, 0x33, 0x03, 0x03, 0x06, 0x0C, 0x18, 0x30, 0x30, 0x33, 0x3F },
	/* 3 */ { 0x1E, 0x33, 0x03, 0x03, 0x0E, 0x03, 0x03, 0x03, 0x03, 0x33, 0x1E },
	/* 4 */ { 0x07, 0x0F, 0x1B, 0x33, 0x33, 0x3F, 0x03, 0x03, 0x03, 0x03, 0x07 },
	/* 5 */ { 0x3F, 0x30, 0x30, 0x3E, 0x03, 0x03, 0x03, 0x03, 0x03, 0x33, 0x1E },
	/* 6 */ { 0x1E, 0x33, 0x30, 0x30, 0x3E, 0x33, 0x33, 0x33, 0x33, 0x33, 0x1E },
	/* 7 */ { 0x3F, 0x03, 0x06, 0x06, 0x0C, 0x0C, 0x18, 0x18, 0x30, 0x30, 0x30 },
	/* 8 */ { 0x1E, 0x33, 0x33, 0x33, 0x1E, 0x33, 0x33, 0x33, 0x33, 0x33, 0x1E },
	/* 9 */ { 0x1E, 0x33, 0x33, 0x33, 0x1F, 0x03, 0x03, 0x03, 0x03, 0x33, 0x1E },
	/* / */ { 0x01, 0x03, 0x06, 0x06, 0x0C, 0x0C, 0x18, 0x18, 0x30, 0x30, 0x20 },
	/* : */ { 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00 },
	/* - */ { 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00 },
	/* + */ { 0x00, 0x00, 0x0C, 0x0C, 0x0C, 0x3F, 0x0C, 0x0C, 0x0C, 0x00, 0x00 },
	/* N */ { 0x33, 0x33, 0x3B, 0x3B, 0x37, 0x37, 0x33, 0x33, 0x33, 0x33, 0x33 },
	/* T */ { 0x3F, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C },
	/* P */ { 0x3E, 0x33, 0x33, 0x33, 0x3E, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30 },
	/* S */ { 0x1E, 0x33, 0x30, 0x30, 0x1E, 0x03, 0x03, 0x03, 0x03, 0x33, 0x1E },
	/* Y */ { 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C },
	/* C */ { 0x1E, 0x33, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x33, 0x1E },
	/* U */ { 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x1E },
	/* ' */ { 0x0C, 0x0C, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	/* " */ { 0x24, 0x24, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	/*   */ { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
};

static int font_char_index(char c)
{
	if (c >= '0' && c <= '9') {
		return c - '0';
	}
	switch (c) {
	case '/': return 10;
	case ':': return 11;
	case '-': return 12;
	case '+': return 13;
	case 'N': case 'n': return 14;
	case 'T': case 't': return 15;
	case 'P': case 'p': return 16;
	case 'S': case 's': return 17;
	case 'Y': case 'y': return 18;
	case 'C': case 'c': return 19;
	case 'U': case 'u': return 20;
	case '\'': return 21;
	case '"':  return 22;
	default:   return 23; /* Space or unknown */
	}
}

static void draw_string(uint16_t *buf, int buf_w, int buf_h,
                        int start_x, int start_y, const char *str,
                        uint16_t text_color)
{
	int cur_x = start_x;
	while (*str) {
		int idx = font_char_index(*str++);
		for (int r = 0; r < 11; r++) {
			int y = start_y + r;
			if (y < 0 || y >= buf_h) {
				continue;
			}
			uint8_t row_bits = font_6x11[idx][r];
			for (int c = 0; c < 6; c++) {
				int x = cur_x + c;
				if (x < 0 || x >= buf_w) {
					continue;
				}
				if ((row_bits >> (5 - c)) & 1) {
					buf[y * buf_w + x] = text_color;
				}
			}
		}
		cur_x += 7; /* 6 pixels glyph + 1 pixel space */
	}
}

static void clear_screen_black(void)
{
	static uint16_t black_line[240] = {0};
	struct display_buffer_descriptor line_desc = {
		.buf_size = sizeof(black_line),
		.width = 240,
		.height = 1,
		.pitch = 240,
		.frame_incomplete = true,
	};
	for (int y = 0; y < 136; y++) {
		line_desc.frame_incomplete = (y < 135);
		display_write(display_dev, 0, y, &line_desc, black_line);
	}
}

static void render_gauge_bar(uint16_t x, uint16_t y, uint32_t pct, uint16_t fill_color)
{
	if (pct > 100) {
		pct = 100;
	}

	uint16_t border_color = rgb565(14, 28, 14);
	uint16_t empty_color  = rgb565(0, 0, 0);
	uint32_t fill_w = (pct * (GAUGE_BAR_W - 2)) / 100;

	for (int row = 0; row < GAUGE_BAR_H; row++) {
		for (int col = 0; col < GAUGE_BAR_W; col++) {
			int idx = row * GAUGE_BAR_W + col;
			if (row == 0 || row == GAUGE_BAR_H - 1 || col == 0 || col == GAUGE_BAR_W - 1) {
				g_draw_buf[idx] = border_color;
			} else {
				int inner_col = col - 1;
				if ((uint32_t)inner_col < fill_w) {
					g_draw_buf[idx] = fill_color;
				} else {
					g_draw_buf[idx] = empty_color;
				}
			}
		}
	}

	struct display_buffer_descriptor desc = {
		.buf_size = GAUGE_BAR_W * GAUGE_BAR_H * sizeof(uint16_t),
		.width = GAUGE_BAR_W,
		.height = GAUGE_BAR_H,
		.pitch = GAUGE_BAR_W,
		.frame_incomplete = false,
	};

	display_write(display_dev, x, y, &desc, g_draw_buf);
}

static void render_time_panel(uint16_t x, uint16_t y,
                              uint8_t month, uint8_t day,
                              uint8_t hour, uint8_t min, uint8_t sec,
                              bool is_synced,
                              uint8_t stage, uint32_t rem_sec)
{
	uint16_t border_col = rgb565(14, 28, 14);
	uint16_t bg_col     = rgb565(0, 0, 0);

	for (int row = 0; row < TIME_PANEL_H; row++) {
		for (int col = 0; col < TIME_PANEL_W; col++) {
			int idx = row * TIME_PANEL_W + col;
			if (row == 0 || row == TIME_PANEL_H - 1 || col == 0 || col == TIME_PANEL_W - 1) {
				g_draw_buf[idx] = border_col;
			} else {
				g_draw_buf[idx] = bg_col;
			}
		}
	}

	char date_str[12];
	char time_str[12];
	if (!is_synced || month == 0) {
		snprintf(date_str, sizeof(date_str), "--/--");
	} else {
		snprintf(date_str, sizeof(date_str), "%02u/%02u", month, day);
	}
	snprintf(time_str, sizeof(time_str), "%02u:%02u:%02u", hour, min, sec);

	/* Top Sync Indicator (at y = 16) */
	if (is_synced) {
		draw_string(g_draw_buf, TIME_PANEL_W, TIME_PANEL_H, 25, 16, "NTP", rgb565(0, 50, 10));
	} else {
		draw_string(g_draw_buf, TIME_PANEL_W, TIME_PANEL_H, 22, 16, "SYNC", rgb565(20, 25, 0));
	}

	/* Line 1: MM/DD (5 chars * 7 - 1 = 34 px wide) -> centered in 70 px: (70 - 34) / 2 = 18 */
	uint16_t date_color = rgb565(0, 56, 31); /* Cool Mint / Cyan */
	draw_string(g_draw_buf, TIME_PANEL_W, TIME_PANEL_H, 18, 44, date_str, date_color);

	/* Line 2: HH:MM:SS (8 chars * 7 - 1 = 55 px wide) -> centered in 70 px: (70 - 55) / 2 = 8 */
	uint16_t time_color = rgb565(31, 55, 0); /* Warm Amber / Gold */
	draw_string(g_draw_buf, TIME_PANEL_W, TIME_PANEL_H, 8, 72, time_str, time_color);

	/* Bottom Tag: Stage Remaining Countdown (at y = 104) replacing UTC+8 */
	uint16_t stage_col;
	switch (stage) {
	case PROFILE_STAGE_LOW:
		stage_col = rgb565(9, 49, 31);   /* Cyan / Sky Blue (Stage 1: Low)   */
		break;
	case PROFILE_STAGE_UP:
		stage_col = rgb565(31, 46, 10);  /* Amber / Gold (Stage 2: Ramp Up)  */
		break;
	case PROFILE_STAGE_HIGH:
		stage_col = rgb565(16, 50, 16);  /* Vibrant Green (Stage 3: High)    */
		break;
	case PROFILE_STAGE_DROP:
		stage_col = rgb565(31, 14, 14);  /* Coral Red (Stage 4: Drop)        */
		break;
	default:
		stage_col = rgb565(14, 28, 14);  /* Dim Grey (Idle)                  */
		break;
	}

	char rem_str[16];
	if (stage != PROFILE_STAGE_IDLE) {
		uint32_t mm = rem_sec / 60;
		uint32_t ss = rem_sec % 60;
		if (mm > 99) {
			mm = 99;
		}
		snprintf(rem_str, sizeof(rem_str), "%02u' %02u''", mm, ss);
	} else {
		snprintf(rem_str, sizeof(rem_str), "--' --''");
	}
	int str_len = strlen(rem_str);
	int text_w = str_len * 7 - 1;
	int start_x = (TIME_PANEL_W - text_w) / 2;
	if (start_x < 2) {
		start_x = 2;
	}
	draw_string(g_draw_buf, TIME_PANEL_W, TIME_PANEL_H, start_x, 104, rem_str, stage_col);

	struct display_buffer_descriptor desc = {
		.buf_size = TIME_PANEL_W * TIME_PANEL_H * sizeof(uint16_t),
		.width = TIME_PANEL_W,
		.height = TIME_PANEL_H,
		.pitch = TIME_PANEL_W,
		.frame_incomplete = false,
	};

	display_write(display_dev, x, y, &desc, g_draw_buf);
}

static void display_thread_fn(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	if (!display_dev || !device_is_ready(display_dev)) {
		printk("[APP_CPU] Display device not ready!\n");
		return;
	}

	/* Flush GRAM to solid black BEFORE enabling backlight to prevent random pixel snow */
	clear_screen_black();
	display_blanking_off(display_dev);

	if (bl_spec.port && device_is_ready(bl_spec.port)) {
		gpio_pin_configure_dt(&bl_spec, GPIO_OUTPUT_ACTIVE);
		gpio_pin_set_dt(&bl_spec, 1);
		printk("[APP_CPU] LCD Backlight enabled on GPIO %d\n", bl_spec.pin);
	}
	printk("[APP_CPU] Display engine initialized (Stacked 3-bar gauge + Right Time Widget)\n");

	/* Bar colors (RGB565) */
	uint16_t color_pwm   = rgb565(0, 63, 31); /* Top: Cyan (PWM Target %) */
	uint16_t color_fg    = rgb565(0, 63, 0);  /* Mid: Vibrant Green (FG RPM %) */
	uint16_t color_curr  = rgb565(31, 40, 0); /* Bot: Orange/Amber (Current mA %, max 250mA) */

	static uint32_t s_smooth_pwm = 0;
	static uint32_t s_smooth_fg = 0;
	static uint32_t s_smooth_curr = 0;

	while (1) {
		uint32_t pwm_pct = g_disp_pwm_pct;
		uint32_t rpm = g_disp_rpm;
		int32_t cur_ma = g_disp_current_ma;

		uint32_t fg_pct = (rpm * 100) / CONFIG_APP_MAX_FG_RPM;
		uint32_t cur_mag = (uint32_t)(cur_ma < 0 ? -cur_ma : cur_ma);
		uint32_t curr_pct = (cur_mag * 100) / APP_MAX_CURRENT_MA;
		if (curr_pct > 100) {
			curr_pct = 100;
		}

		/* Smooth low-pass visual animation to eliminate jumping bars on LCD */
		s_smooth_pwm  = (s_smooth_pwm * 3 + pwm_pct) / 4;
		s_smooth_fg   = (s_smooth_fg * 3 + fg_pct) / 4;
		s_smooth_curr = (s_smooth_curr * 3 + curr_pct) / 4;

		/* 3 Stacked Bars (Left): No spacing between them */
		/* Bar 0 (PWM Target): Y = 2..45 (H = 44) */
		render_gauge_bar(GAUGE_BAR_X, 2, s_smooth_pwm, color_pwm);
		/* Bar 1 (FG Speed):   Y = 46..89 (H = 44) */
		render_gauge_bar(GAUGE_BAR_X, 46, s_smooth_fg, color_fg);
		/* Bar 2 (Current mA): Y = 90..133 (H = 44) */
		render_gauge_bar(GAUGE_BAR_X, 90, s_smooth_curr, color_curr);

		/* Compute continuous local wall-clock time so screen clock never freezes */
		int64_t now_ms = k_uptime_get();
		uint8_t disp_hour;
		uint8_t disp_min;
		uint8_t disp_sec;
		uint32_t cur_sec_of_day;

		if (g_time_synced) {
			int64_t ms_since_sync = now_ms - g_time_sync_uptime_ms;
			if (ms_since_sync < 0) {
				ms_since_sync = 0;
			}
			uint64_t total_day_ms = (uint64_t)g_time_sec_of_day * 1000ULL +
			                        (uint64_t)g_time_ms_part +
			                        (uint64_t)ms_since_sync;
			cur_sec_of_day = (uint32_t)((total_day_ms / 1000ULL) % 86400ULL);
			disp_hour = (uint8_t)((cur_sec_of_day / 3600U) % 24U);
			disp_min  = (uint8_t)((cur_sec_of_day % 3600U) / 60U);
			disp_sec  = (uint8_t)(cur_sec_of_day % 60U);
		} else {
			/* Before NTP sync, show elapsed uptime HH:MM:SS */
			uint32_t uptime_sec = (uint32_t)(now_ms / 1000ULL);
			cur_sec_of_day = uptime_sec;
			disp_hour = (uint8_t)((uptime_sec / 3600U) % 24U);
			disp_min  = (uint8_t)((uptime_sec % 3600U) / 60U);
			disp_sec  = (uint8_t)(uptime_sec % 60U);
		}

		/* Compute active profile stage and countdown of remaining seconds in stage */
		uint32_t t1 = g_prof_cfg.t_low_sec;
		uint32_t t2 = g_prof_cfg.t_up_sec;
		uint32_t t3 = g_prof_cfg.t_high_sec;
		uint32_t t4 = g_prof_cfg.t_drop_sec;
		uint32_t total = t1 + t2 + t3 + t4;

		uint8_t stage = g_prof_stage;
		uint32_t rem_sec = 0;
		if (stage != PROFILE_STAGE_IDLE && total > 0) {
			uint32_t elapsed_sec = cur_sec_of_day % total;
			if (elapsed_sec < t1) {
				stage = PROFILE_STAGE_LOW;
				rem_sec = t1 - elapsed_sec;
			} else if (elapsed_sec < t1 + t2) {
				stage = PROFILE_STAGE_UP;
				rem_sec = (t1 + t2) - elapsed_sec;
			} else if (elapsed_sec < t1 + t2 + t3) {
				stage = PROFILE_STAGE_HIGH;
				rem_sec = (t1 + t2 + t3) - elapsed_sec;
			} else {
				stage = PROFILE_STAGE_DROP;
				rem_sec = total - elapsed_sec;
			}
		}

		/* Right Time & Stage Countdown Widget: Y = 2..133, X = 166..235 */
		render_time_panel(TIME_PANEL_X, TIME_PANEL_Y,
		                  g_time_month, g_time_day,
		                  disp_hour, disp_min, disp_sec,
		                  g_time_synced,
		                  stage, rem_sec);

		k_msleep(100); /* 10 Hz refresh rate */
	}
}

K_THREAD_DEFINE(thread_speed, THREAD_STACK_SIZE, motor_speed_thread_fn,
                NULL, NULL, NULL, PRIORITY_HIGH, 0, 0);

K_THREAD_DEFINE(thread_aggregator, THREAD_STACK_SIZE, sensor_aggregator_thread_fn,
                NULL, NULL, NULL, PRIORITY_MEDIUM, 0, 0);

K_THREAD_DEFINE(thread_display, DISPLAY_STACK_SIZE, display_thread_fn,
                NULL, NULL, NULL, PRIORITY_DISPLAY, 0, 0);

int main(void)
{
	const struct device *const timer_dev = DEVICE_DT_GET(TIMER_NODE);
	int err;

	ipm_dev = DEVICE_DT_GET(DT_NODELABEL(ipm0));

	printk("=================================================================\n");
	printk("Waveshare ESP32-S3-GEEK: APP_CPU (Core 1) Real-Time Motor Engine\n");
	printk("Target: esp32s3_geek/esp32s3/appcpu\n");
	printk("=================================================================\n");

	if (!ipm_dev || !device_is_ready(ipm_dev)) {
		/* IPM not ready */
	} else {
		ipm_register_callback(ipm_dev, app_ipm_callback, NULL);
		ipm_set_enabled(ipm_dev, 1);
		printk("[APP_CPU] IPM ready: TX ch %d, %d | RX ch %d (time sync)\n",
		       IPM_CHANNEL_APP_TELEMETRY, IPM_CHANNEL_APP_PRINT, IPM_CHANNEL_TIME_SYNC);
	}

	if (!device_is_ready(timer_dev)) {
		printk("[APP_CPU] FATAL: Counter device %s is not ready!\n", timer_dev->name);
		return -ENODEV;
	}

	struct counter_top_cfg top_cfg = {
		.ticks = counter_us_to_ticks(timer_dev, TIMER_INTERVAL_US),
		.callback = timer_top_subroutine,
		.user_data = NULL,
		.flags = 0,
	};

	err = counter_set_top_value(timer_dev, &top_cfg);
	if (err) {
		printk("[APP_CPU] FATAL: Failed to set counter top: %d\n", err);
		return err;
	}

	err = counter_start(timer_dev);
	if (err) {
		printk("[APP_CPU] FATAL: Failed to start counter: %d\n", err);
		return err;
	}

	printk("[APP_CPU] Real-time engine active at %d Hz.\n", CONFIG_APP_SAMPLE_RATE_HZ);

	while (1) {
		k_sleep(K_SECONDS(30));
		printk("[APP_CPU Supervisor] Uptime: %lld ms\n", k_uptime_get());
	}

	return 0;
}
