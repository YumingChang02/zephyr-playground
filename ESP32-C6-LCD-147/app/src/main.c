/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Waveshare ESP32-C6-LCD-1.47 Real-Time Dual-Loop Motor Controller & Telemetry System
 * Unified Single-Core RISC-V Architecture with Multithreaded Zephyr RTOS
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/display.h>
#include <zephyr/fs/fs.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/sntp.h>
#include <zephyr/sys/printk.h>
#include <zephyr/spinlock.h>
#include <ff.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <time.h>

#include "telemetry.h"

/* POSIX socket aliases */
#define socket(f, t, p)             zsock_socket(f, t, p)
#define bind(s, a, l)               zsock_bind(s, a, l)
#define listen(s, b)                zsock_listen(s, b)
#define accept(s, a, l)             zsock_accept(s, a, l)
#define send(s, b, l, f)            zsock_send(s, b, l, f)
#define recv(s, b, l, f)            zsock_recv(s, b, l, f)
#define close(s)                    zsock_close(s)
#define select(n, r, w, e, t)       zsock_select(n, r, w, e, t)
#define poll(p, n, t)               zsock_poll(p, n, t)
#define setsockopt(s, l, o, v, len) zsock_setsockopt(s, l, o, v, len)

/* ========================================================================= */
/* Application Configuration & Priorities                                    */
/* ========================================================================= */
#ifndef CONFIG_APP_SAMPLE_RATE_HZ
#define CONFIG_APP_SAMPLE_RATE_HZ       3
#endif
#ifndef CONFIG_APP_MOTOR_PULSES_PER_REV
#define CONFIG_APP_MOTOR_PULSES_PER_REV 6
#endif
#ifndef CONFIG_APP_MAX_FG_RPM
#define CONFIG_APP_MAX_FG_RPM           5500
#endif
#ifndef CONFIG_APP_TIMEZONE_OFFSET_SEC
#define CONFIG_APP_TIMEZONE_OFFSET_SEC  (8 * 3600)  /* UTC+8 (28,800 s) */
#endif

#define TIMER_INTERVAL_US               (1000000 / CONFIG_APP_SAMPLE_RATE_HZ)

/* Thread Priorities (Preemptive) */
#define PRIORITY_MOTOR_SPEED            2   /* Real-time PWM & FG Counter */
#define PRIORITY_SENSOR                 4   /* INA219 Aggregator */
#define PRIORITY_DISPLAY                5   /* ST7789 LCD Engine */
#define PRIORITY_SD_LOGGER              7   /* FAT32 File Logger */
#define PRIORITY_WIFI_HTTP              8   /* Web Server & SSE Streaming */
#define PRIORITY_DOWNLOAD               9   /* Background Non-blocking Download */
#define PRIORITY_NTP                    10  /* Background Clock Sync */

#define STACK_MOTOR_SPEED               3072
#define STACK_SENSOR                    3072
#define STACK_DISPLAY                   4096
#define STACK_SD_LOGGER                 4096
#define STACK_WIFI_HTTP                 4096
#define STACK_DOWNLOAD                  4096
#define STACK_NTP                       3072

#define MAX_SSE_CLIENTS                 4
#define HTTP_PORT                       80

/* ========================================================================= */
/* Device Tree Bindings & Peripherals                                        */
/* ========================================================================= */
#define TIMER_NODE      DT_ALIAS(app_timer)
#define PWM_MOTOR_NODE  DT_ALIAS(motor_pwm)
#define FG_PIN_NODE     DT_ALIAS(fg_pin)
#define LCD_BL_NODE     DT_ALIAS(lcd_bl)

static const struct pwm_dt_spec pwm_motor = PWM_DT_SPEC_GET(PWM_MOTOR_NODE);
static const struct gpio_dt_spec fg_spec  = GPIO_DT_SPEC_GET(FG_PIN_NODE, gpios);
static const struct gpio_dt_spec bl_spec  = GPIO_DT_SPEC_GET(LCD_BL_NODE, gpios);
static const struct device *display_dev   = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

/* ========================================================================= */
/* Shared Thread State, Queues & Synchronization                            */
/* ========================================================================= */
typedef struct {
	uint32_t pulse_count;
	uint32_t raw_rpm;
	uint32_t rpm;
	uint32_t pwm_pct;
	uint8_t  stage;
	uint16_t elapsed_sec;
	uint16_t total_sec;
	int64_t  uptime_ms;
	uint32_t sample_id;
} motor_speed_sample_t;

K_MSGQ_DEFINE(motor_speed_msgq, sizeof(motor_speed_sample_t), 4, 4);
K_MSGQ_DEFINE(csv_msgq, sizeof(telemetry_full_t), 8, 4);
static K_SEM_DEFINE(timer_tick_sem, 0, 1);

/* Spinlocks */
static struct k_spinlock telemetry_spinlock;
static telemetry_full_t latest_telemetry;
static bool has_telemetry = false;

static struct k_spinlock time_spinlock;
static int64_t ntp_epoch_sec = 0;
static int64_t ntp_sync_uptime = 0;
static bool ntp_synced = false;
static K_SEM_DEFINE(ntp_trigger_sem, 0, 1);

/* Mutexes */
static K_MUTEX_DEFINE(sdcard_fs_mutex);
static K_MUTEX_DEFINE(csv_path_mutex);
static char active_csv_filename[64] = "";

/* Active Profile Configuration */
static struct k_spinlock profile_spinlock;
static pwm_profile_ctrl_t g_prof_cfg = {
	.t_low_sec = DEFAULT_PROF_T_LOW,
	.p_low_pct = DEFAULT_PROF_P_LOW,
	.t_up_sec = DEFAULT_PROF_T_UP,
	.t_high_sec = DEFAULT_PROF_T_HIGH,
	.p_high_pct = DEFAULT_PROF_P_HIGH,
	.t_drop_sec = DEFAULT_PROF_T_DROP,
	.action = PWM_ACTION_START_PROFILE,
	.manual_pwm = 0,
};
static volatile uint8_t  g_prof_stage = PROFILE_STAGE_LOW;
static volatile uint16_t g_prof_elapsed_sec = 0;
static volatile uint16_t g_prof_total_sec = DEFAULT_PROF_TOTAL;
static volatile uint32_t g_active_pwm_pct = 0;

/* Real-Time Display Shared Values */
static volatile uint32_t g_disp_pwm_pct = 0;
static volatile uint32_t g_disp_rpm = 0;
static volatile int32_t  g_disp_current_ma = 0;

/* Wi-Fi & SSE State */
static volatile bool wifi_connected = false;
static volatile bool wifi_has_ip = false;
static char wifi_ip_addr[16] = "0.0.0.0";
static volatile int64_t last_dhcp_log = 0;

typedef struct {
	int fd;
	uint32_t last_sent_id;
} sse_client_t;
static sse_client_t sse_clients[MAX_SSE_CLIENTS];

/* Background CSV Download State */
static K_SEM_DEFINE(download_start_sem, 0, 1);
static volatile int g_download_client_fd = -1;
static volatile bool g_download_in_progress = false;

/* SD Card & FAT32 State */
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = NULL,
	.mnt_point = "/SD:",
};
static FATFS fat_fs;
static bool sdcard_mounted = false;
static struct fs_file_t g_csv_file;
static bool g_csv_file_is_open = false;

/* ========================================================================= */
/* NTP & Time Formatting Helpers                                             */
/* ========================================================================= */
static void get_formatted_time(char *iso_buf, size_t iso_len, char *date_buf, size_t date_len)
{
	k_spinlock_key_t key = k_spin_lock(&time_spinlock);
	bool synced = ntp_synced;
	int64_t epoch = ntp_epoch_sec;
	int64_t uptime_ref = ntp_sync_uptime;
	k_spin_unlock(&time_spinlock, key);

	if (!synced) {
		if (iso_buf && iso_len > 0) {
			snprintf(iso_buf, iso_len, "NTP_PENDING");
		}
		if (date_buf && date_len > 0) {
			date_buf[0] = '\0';
		}
		return;
	}

	int64_t now_sec = epoch + (k_uptime_get() - uptime_ref) / 1000;
	time_t t = (time_t)(now_sec + CONFIG_APP_TIMEZONE_OFFSET_SEC);
	struct tm tm_info;
	gmtime_r(&t, &tm_info);

	int year = (int)(tm_info.tm_year % 10000) + 1900;
	int mon = (int)(tm_info.tm_mon % 12) + 1;
	int day = (int)(tm_info.tm_mday % 32);
	int hour = (int)(tm_info.tm_hour % 24);
	int min = (int)(tm_info.tm_min % 60);
	int sec = (int)(tm_info.tm_sec % 60);

	if (iso_buf && iso_len >= 26) {
		snprintf(iso_buf, iso_len, "%04d-%02d-%02dT%02d:%02d:%02d+08:00",
		         year, mon, day, hour, min, sec);
	}
	if (date_buf && date_len >= 11) {
		snprintf(date_buf, date_len, "%04d-%02d-%02d",
		         year, mon, day);
	}
}

/* Helper to reliably send all bytes across a TCP socket */
static int send_all(int sock, const void *buf, size_t len)
{
	const char *p = (const char *)buf;
	while (len > 0) {
		int sent = send(sock, p, len, 0);
		if (sent <= 0) {
			return -1;
		}
		p += sent;
		len -= sent;
	}
	return 0;
}

/* ========================================================================= */
/* Thread 1: Real-Time Motor Speed & Profile Engine (Priority 2)             */
/* ========================================================================= */
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

static void timer_top_subroutine(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	k_sem_give(&timer_tick_sem);
}

static void motor_speed_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[Thread 1: Motor Engine] Started on RISC-V HP Core (Priority %d)\n", PRIORITY_MOTOR_SPEED);

	int64_t last_time_ms = k_uptime_get();
	uint32_t duty_cycle_pct = 0;
	g_disp_pwm_pct = duty_cycle_pct;

	if (fg_spec.port && device_is_ready(fg_spec.port)) {
		gpio_pin_configure_dt(&fg_spec, GPIO_INPUT);
		gpio_init_callback(&fg_gpio_cb, fg_gpio_isr, BIT(fg_spec.pin));
		gpio_add_callback(fg_spec.port, &fg_gpio_cb);
		gpio_pin_interrupt_configure_dt(&fg_spec, GPIO_INT_EDGE_FALLING);
		printk("[Thread 1] FG Counter initialized on GPIO %d\n", fg_spec.pin);
	}

	if (pwm_motor.dev && device_is_ready(pwm_motor.dev)) {
		pwm_set_dt(&pwm_motor, pwm_motor.period, 0);
		printk("[Thread 1] Hardware LEDC PWM initialized on period %u ns\n", (uint32_t)pwm_motor.period);
	}

	uint32_t sample_seq = 0;
	uint32_t ema_rpm = 0;

	while (1) {
		k_sem_take(&timer_tick_sem, K_FOREVER);

		int64_t current_time_ms = k_uptime_get();
		int64_t elapsed_ms = current_time_ms - last_time_ms;
		if (elapsed_ms <= 0) {
			elapsed_ms = 1;
		}
		last_time_ms = current_time_ms;

		unsigned int key = irq_lock();
		uint32_t current_pulses = g_fg_pulse_counter;
		g_fg_pulse_counter = 0;
		irq_unlock(key);

		g_total_pulses += current_pulses;

		/* Compute second of day for 00:00:00 alignment (or relative uptime if pending) */
		uint32_t cur_sec_of_day;
		k_spinlock_key_t tkey = k_spin_lock(&time_spinlock);
		bool synced = ntp_synced;
		int64_t epoch = ntp_epoch_sec;
		int64_t uptime_ref = ntp_sync_uptime;
		k_spin_unlock(&time_spinlock, tkey);

		if (synced) {
			int64_t now_sec = epoch + (current_time_ms - uptime_ref) / 1000;
			time_t t_local = (time_t)(now_sec + CONFIG_APP_TIMEZONE_OFFSET_SEC);
			struct tm tm_val;
			gmtime_r(&t_local, &tm_val);
			cur_sec_of_day = (uint32_t)tm_val.tm_hour * 3600U +
			                 (uint32_t)tm_val.tm_min * 60U +
			                 (uint32_t)tm_val.tm_sec;
		} else {
			/* Before NTP lock, cycle profile from system boot uptime */
			cur_sec_of_day = (uint32_t)(current_time_ms / 1000);
		}

		/* Read current profile configuration */
		k_spinlock_key_t pkey = k_spin_lock(&profile_spinlock);
		pwm_profile_ctrl_t cfg = g_prof_cfg;
		k_spin_unlock(&profile_spinlock, pkey);

		uint32_t target_pwm = 0;
		uint8_t stage = PROFILE_STAGE_IDLE;
		uint16_t prof_elapsed = 0;
		uint32_t t1 = cfg.t_low_sec;
		uint32_t t2 = cfg.t_up_sec;
		uint32_t t3 = cfg.t_high_sec;
		uint32_t t4 = cfg.t_drop_sec;
		uint32_t total = t1 + t2 + t3 + t4;
		g_prof_total_sec = (uint16_t)total;

		if (cfg.action == PWM_ACTION_START_PROFILE && total > 0) {
			uint32_t phase_sec = cur_sec_of_day % total;
			prof_elapsed = (uint16_t)phase_sec;

			if (phase_sec < t1) {
				stage = PROFILE_STAGE_LOW;
				target_pwm = cfg.p_low_pct;
			} else if (phase_sec < t1 + t2) {
				stage = PROFILE_STAGE_UP;
				uint32_t dt = phase_sec - t1;
				if (t2 > 0) {
					target_pwm = cfg.p_low_pct + ((cfg.p_high_pct - cfg.p_low_pct) * dt) / t2;
				} else {
					target_pwm = cfg.p_high_pct;
				}
			} else if (phase_sec < t1 + t2 + t3) {
				stage = PROFILE_STAGE_HIGH;
				target_pwm = cfg.p_high_pct;
			} else {
				stage = PROFILE_STAGE_DROP;
				uint32_t dt = phase_sec - (t1 + t2 + t3);
				if (t4 > 0) {
					target_pwm = cfg.p_high_pct - ((cfg.p_high_pct - cfg.p_low_pct) * dt) / t4;
				} else {
					target_pwm = cfg.p_low_pct;
				}
			}
		} else if (cfg.action == PWM_ACTION_MANUAL_PWM) {
			stage = PROFILE_STAGE_IDLE;
			prof_elapsed = 0;
			target_pwm = cfg.manual_pwm;
		} else {
			stage = PROFILE_STAGE_IDLE;
			prof_elapsed = 0;
			target_pwm = 0;
		}

		if (target_pwm > 100) target_pwm = 100;
		duty_cycle_pct = target_pwm;
		g_active_pwm_pct = duty_cycle_pct;
		g_prof_stage = stage;
		g_prof_elapsed_sec = prof_elapsed;

		/* Apply hardware PWM */
		if (pwm_motor.dev && device_is_ready(pwm_motor.dev)) {
			uint32_t pulse_ns = (uint32_t)(((uint64_t)pwm_motor.period * duty_cycle_pct) / 100ULL);
			pwm_set_dt(&pwm_motor, pwm_motor.period, pulse_ns);
		}
		g_disp_pwm_pct = duty_cycle_pct;

		/* Calculate Motor RPM */
		uint32_t raw_rpm = (uint32_t)(((uint64_t)current_pulses * 60000ULL) /
		                              (CONFIG_APP_MOTOR_PULSES_PER_REV * (uint64_t)elapsed_ms));
		if (ema_rpm == 0) {
			ema_rpm = raw_rpm;
		} else {
			ema_rpm = (ema_rpm * 3 + raw_rpm) / 4;
		}
		g_disp_rpm = ema_rpm;

		motor_speed_sample_t sample = {
			.pulse_count = current_pulses,
			.raw_rpm = raw_rpm,
			.rpm = ema_rpm,
			.pwm_pct = duty_cycle_pct,
			.stage = stage,
			.elapsed_sec = prof_elapsed,
			.total_sec = (uint16_t)total,
			.uptime_ms = current_time_ms,
			.sample_id = ++sample_seq,
		};
		k_msgq_put(&motor_speed_msgq, &sample, K_NO_WAIT);
	}
}

/* ========================================================================= */
/* Thread 2: INA219 Sensor Aggregator (Priority 4)                           */
/* ========================================================================= */
static void sensor_aggregator_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[Thread 2: Sensor Aggregator] Started (Priority %d)\n", PRIORITY_SENSOR);

	const struct device *const ina219_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(ina219));
	if (ina219_dev && device_is_ready(ina219_dev)) {
		printk("[Thread 2] Real INA219 sensor found at 0x40 on i2c0\n");
	} else {
		printk("[Thread 2] INA219 not detected, using simulation\n");
	}

	while (1) {
		motor_speed_sample_t speed;
		k_msgq_get(&motor_speed_msgq, &speed, K_FOREVER);

		int32_t bus_mv = 12000;
		int32_t current_ma = 0;
		int32_t temp_mc = 38500;

		if (ina219_dev && device_is_ready(ina219_dev)) {
			struct sensor_value v_bus, v_curr;
			if (sensor_sample_fetch(ina219_dev) == 0) {
				sensor_channel_get(ina219_dev, SENSOR_CHAN_VOLTAGE, &v_bus);
				sensor_channel_get(ina219_dev, SENSOR_CHAN_CURRENT, &v_curr);
				bus_mv = v_bus.val1 * 1000 + v_bus.val2 / 1000;
				current_ma = v_curr.val1 * 1000 + v_curr.val2 / 1000;
			}
		} else {
			/* Simulation proportional to speed */
			current_ma = 100 + (int32_t)((speed.rpm * 120) / CONFIG_APP_MAX_FG_RPM);
			bus_mv = 12050 - (speed.rpm / 200);
			temp_mc = 38500 + (speed.rpm * 2);
		}

		g_disp_current_ma = current_ma;

		telemetry_full_t full = {
			.sample_id = speed.sample_id,
			.motor_rpm = speed.rpm,
			.temp_mc = temp_mc,
			.bus_mv = (uint32_t)bus_mv,
			.current_ma = current_ma,
			.uptime_ms = speed.uptime_ms,
			.pwm_pct = (uint8_t)speed.pwm_pct,
			.profile_stage = speed.stage,
			.profile_elapsed_sec = speed.elapsed_sec,
			.profile_total_sec = speed.total_sec,
		};
		get_formatted_time(full.iso_time, sizeof(full.iso_time),
		                   full.date_str, sizeof(full.date_str));

		/* Enqueue to SD Card logger only after real wall-clock time is locked by NTP */
		if (ntp_synced) {
			k_msgq_put(&csv_msgq, &full, K_NO_WAIT);
		}

		/* Update latest telemetry for SSE clients */
		k_spinlock_key_t key = k_spin_lock(&telemetry_spinlock);
		latest_telemetry = full;
		has_telemetry = true;
		k_spin_unlock(&telemetry_spinlock, key);
	}
}

/* ========================================================================= */
/* Thread 3: ST7789 LCD Display Engine (172x320) (Priority 5)               */
/* ========================================================================= */
#define LCD_W           320
#define LCD_H           172

#define GAUGE_BAR_X     2
#define GAUGE_BAR_W     220
#define GAUGE_BAR_H     54

#define TIME_PANEL_X    224
#define TIME_PANEL_Y    2
#define TIME_PANEL_W    94
#define TIME_PANEL_H    168

static uint16_t g_bar_buf[GAUGE_BAR_W * GAUGE_BAR_H];
static uint16_t g_time_buf[TIME_PANEL_W * TIME_PANEL_H];

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
	return ((r & 0x1F) << 11) | ((g & 0x3F) << 5) | (b & 0x1F);
}

/* 6x11 Monochrome Font with Numbers, Delimiters and Quotes */
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
	default:   return 23;
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
			if (y < 0 || y >= buf_h) continue;
			uint8_t row_bits = font_6x11[idx][r];
			for (int c = 0; c < 6; c++) {
				int x = cur_x + c;
				if (x < 0 || x >= buf_w) continue;
				if ((row_bits >> (5 - c)) & 1) {
					buf[y * buf_w + x] = text_color;
				}
			}
		}
		cur_x += 7;
	}
}

static void render_gauge_bar(int x, int y, uint32_t pct, uint16_t bar_color)
{
	if (pct > 100) pct = 100;
	int fill_w = (int)(((uint64_t)pct * (GAUGE_BAR_W - 4)) / 100);

	for (int row = 0; row < GAUGE_BAR_H; row++) {
		for (int col = 0; col < GAUGE_BAR_W; col++) {
			int idx = row * GAUGE_BAR_W + col;
			if (row == 0 || row == GAUGE_BAR_H - 1 || col == 0 || col == GAUGE_BAR_W - 1) {
				g_bar_buf[idx] = rgb565(8, 16, 8); /* Subtle border */
			} else if (col >= 2 && col < 2 + fill_w && row >= 2 && row < GAUGE_BAR_H - 2) {
				g_bar_buf[idx] = bar_color;
			} else {
				g_bar_buf[idx] = rgb565(0, 0, 0); /* Pure Black */
			}
		}
	}

	struct display_buffer_descriptor desc = {
		.buf_size = sizeof(g_bar_buf),
		.width = GAUGE_BAR_W,
		.height = GAUGE_BAR_H,
		.pitch = GAUGE_BAR_W,
		.frame_incomplete = false,
	};
	display_write(display_dev, x, y, &desc, g_bar_buf);
}

static void render_time_panel(int x, int y, uint8_t mon, uint8_t day,
                              uint8_t hour, uint8_t min, uint8_t sec,
                              uint8_t stage, uint32_t rem_sec)
{
	for (int i = 0; i < TIME_PANEL_W * TIME_PANEL_H; i++) {
		g_time_buf[i] = rgb565(2, 4, 2); /* Deep Slate Background */
	}

	/* Top: Date MM/DD (or "--/--" if NTP not synced yet) */
	char date_str[12];
	if (mon == 0 || day == 0) {
		snprintf(date_str, sizeof(date_str), "--/--");
	} else {
		snprintf(date_str, sizeof(date_str), "%02u/%02u", mon, day);
	}
	draw_string(g_time_buf, TIME_PANEL_W, TIME_PANEL_H, 29, 36, date_str, rgb565(0, 56, 31));

	/* Mid: HH:MM:SS */
	char time_str[12];
	snprintf(time_str, sizeof(time_str), "%02u:%02u:%02u", hour, min, sec);
	draw_string(g_time_buf, TIME_PANEL_W, TIME_PANEL_H, 19, 78, time_str, rgb565(31, 55, 0));

	/* Bottom: Stage Remaining MM' SS'' */
	uint16_t stage_col;
	switch (stage) {
	case PROFILE_STAGE_LOW:  stage_col = rgb565(9, 49, 31);  break; /* Cyan */
	case PROFILE_STAGE_UP:   stage_col = rgb565(31, 46, 10); break; /* Amber */
	case PROFILE_STAGE_HIGH: stage_col = rgb565(16, 50, 16); break; /* Green */
	case PROFILE_STAGE_DROP: stage_col = rgb565(31, 14, 14); break; /* Red */
	default:                 stage_col = rgb565(14, 28, 14); break; /* Grey */
	}

	char rem_str[16];
	if (stage != PROFILE_STAGE_IDLE) {
		uint32_t mm = rem_sec / 60;
		uint32_t ss = rem_sec % 60;
		if (mm > 99) mm = 99;
		snprintf(rem_str, sizeof(rem_str), "%02u' %02u''", mm, ss);
	} else {
		snprintf(rem_str, sizeof(rem_str), "--' --''");
	}
	int str_len = strlen(rem_str);
	int text_w = str_len * 7 - 1;
	int start_x = (TIME_PANEL_W - text_w) / 2;
	if (start_x < 2) start_x = 2;
	draw_string(g_time_buf, TIME_PANEL_W, TIME_PANEL_H, start_x, 120, rem_str, stage_col);

	struct display_buffer_descriptor desc = {
		.buf_size = sizeof(g_time_buf),
		.width = TIME_PANEL_W,
		.height = TIME_PANEL_H,
		.pitch = TIME_PANEL_W,
		.frame_incomplete = false,
	};
	display_write(display_dev, x, y, &desc, g_time_buf);
}

static void clear_screen_black(void)
{
	memset(g_bar_buf, 0, sizeof(g_bar_buf));
	const int strip_h = 20;
	struct display_buffer_descriptor desc = {
		.buf_size = LCD_W * strip_h * sizeof(uint16_t),
		.width = LCD_W,
		.height = strip_h,
		.pitch = LCD_W,
		.frame_incomplete = false,
	};

	for (int y = 0; y < LCD_H; y += strip_h) {
		int h = (y + strip_h <= LCD_H) ? strip_h : (LCD_H - y);
		desc.height = h;
		desc.buf_size = LCD_W * h * sizeof(uint16_t);
		display_write(display_dev, 0, y, &desc, g_bar_buf);
	}
}

static void display_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[Thread 3: LCD Engine] Started (Priority %d)\n", PRIORITY_DISPLAY);

	if (!display_dev || !device_is_ready(display_dev)) {
		printk("[Thread 3] Display device not ready!\n");
		return;
	}

	/* 1. Flush full 320x172 screen with solid black before turning on backlight or unblanking */
	clear_screen_black();
	printk("[Thread 3] LCD screen flushed to solid black\n");

	/* 2. Unblank the display controller */
	display_blanking_off(display_dev);

	/* 3. Turn on backlight cleanly */
	if (bl_spec.port && device_is_ready(bl_spec.port)) {
		gpio_pin_configure_dt(&bl_spec, GPIO_OUTPUT_ACTIVE);
		gpio_pin_set_dt(&bl_spec, 1);
		printk("[Thread 3] LCD Backlight enabled on GPIO %d\n", bl_spec.pin);
	}

	uint16_t color_pwm   = rgb565(0, 63, 31); /* Top: Cyan (PWM Target %) */
	uint16_t color_fg    = rgb565(0, 63, 0);  /* Mid: Green (FG RPM %) */
	uint16_t color_curr  = rgb565(31, 40, 0); /* Bot: Amber (Current mA %) */

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
		if (curr_pct > 100) curr_pct = 100;

		s_smooth_pwm  = (s_smooth_pwm * 3 + pwm_pct) / 4;
		s_smooth_fg   = (s_smooth_fg * 3 + fg_pct) / 4;
		s_smooth_curr = (s_smooth_curr * 3 + curr_pct) / 4;

		/* 3 Stacked Bars on Left: Y = 2..55, 58..111, 114..167 */
		render_gauge_bar(GAUGE_BAR_X, 2, s_smooth_pwm, color_pwm);
		render_gauge_bar(GAUGE_BAR_X, 58, s_smooth_fg, color_fg);
		render_gauge_bar(GAUGE_BAR_X, 114, s_smooth_curr, color_curr);

		/* Compute continuous local time (real wall-clock if NTP locked, uptime if pending) */
		int64_t now_ms = k_uptime_get();
		k_spinlock_key_t key = k_spin_lock(&time_spinlock);
		bool synced = ntp_synced;
		int64_t epoch = ntp_epoch_sec;
		int64_t uptime_ref = ntp_sync_uptime;
		k_spin_unlock(&time_spinlock, key);

		uint8_t disp_mon = 0, disp_day = 0;
		uint8_t disp_hour = 0, disp_min = 0, disp_sec = 0;
		uint32_t cur_sec_of_day;

		if (synced) {
			int64_t now_sec = epoch + (now_ms - uptime_ref) / 1000;
			time_t t_local = (time_t)(now_sec + CONFIG_APP_TIMEZONE_OFFSET_SEC);
			struct tm tm_val;
			gmtime_r(&t_local, &tm_val);
			disp_mon  = (uint8_t)(tm_val.tm_mon + 1);
			disp_day  = (uint8_t)tm_val.tm_mday;
			disp_hour = (uint8_t)tm_val.tm_hour;
			disp_min  = (uint8_t)tm_val.tm_min;
			disp_sec  = (uint8_t)tm_val.tm_sec;
			cur_sec_of_day = (uint32_t)disp_hour * 3600U +
			                 (uint32_t)disp_min * 60U +
			                 (uint32_t)disp_sec;
		} else {
			/* Elapsed uptime before NTP lock */
			uint32_t up_s = (uint32_t)(now_ms / 1000);
			disp_hour = (uint8_t)((up_s / 3600U) % 24U);
			disp_min  = (uint8_t)((up_s % 3600U) / 60U);
			disp_sec  = (uint8_t)(up_s % 60U);
			cur_sec_of_day = up_s;
		}

		/* Compute remaining seconds in active stage */
		k_spinlock_key_t pkey = k_spin_lock(&profile_spinlock);
		pwm_profile_ctrl_t cfg = g_prof_cfg;
		k_spin_unlock(&profile_spinlock, pkey);

		uint32_t t1 = cfg.t_low_sec;
		uint32_t t2 = cfg.t_up_sec;
		uint32_t t3 = cfg.t_high_sec;
		uint32_t t4 = cfg.t_drop_sec;
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

		render_time_panel(TIME_PANEL_X, TIME_PANEL_Y,
		                  disp_mon, disp_day,
		                  disp_hour, disp_min, disp_sec,
		                  stage, rem_sec);

		k_msleep(100); /* 10 Hz refresh */
	}
}

/* ========================================================================= */
/* Thread 4: FAT32 SD Card CSV Logger (Priority 7)                           */
/* ========================================================================= */
static void init_sdcard_fat32(void)
{
	k_mutex_lock(&sdcard_fs_mutex, K_FOREVER);
	if (sdcard_mounted) {
		k_mutex_unlock(&sdcard_fs_mutex);
		return;
	}
	mp.fs_data = &fat_fs;
	int res = fs_mount(&mp);
	if (res == 0) {
		sdcard_mounted = true;
		printk("[Thread 4: SD Card] Mounted successfully at %s\n", mp.mnt_point);
	} else {
		printk("[Thread 4: SD Card] Mount failed (%d)\n", res);
	}
	k_mutex_unlock(&sdcard_fs_mutex);
}

static void prune_old_csv_files(void)
{
	k_mutex_lock(&sdcard_fs_mutex, K_FOREVER);
	if (!sdcard_mounted) {
		k_mutex_unlock(&sdcard_fs_mutex);
		return;
	}

	struct fs_dir_t dirp;
	fs_dir_t_init(&dirp);
	int res = fs_opendir(&dirp, "/SD:");
	if (res != 0) {
		k_mutex_unlock(&sdcard_fs_mutex);
		return;
	}

	char csv_files[32][32];
	int count = 0;
	struct fs_dirent entry;

	while (fs_readdir(&dirp, &entry) == 0 && entry.name[0] != '\0') {
		if (entry.type == FS_DIR_ENTRY_FILE &&
		    strncmp(entry.name, "telemetry_", 10) == 0 &&
		    strstr(entry.name, ".csv") != NULL) {
			if (count < 32) {
				strncpy(csv_files[count], entry.name, sizeof(csv_files[count]) - 1);
				csv_files[count][sizeof(csv_files[count]) - 1] = '\0';
				count++;
			}
		}
	}
	fs_closedir(&dirp);

	/* Keep latest 7 files */
	if (count > 7) {
		for (int i = 0; i < count - 1; i++) {
			for (int j = i + 1; j < count; j++) {
				if (strcmp(csv_files[i], csv_files[j]) > 0) {
					char tmp[32];
					strcpy(tmp, csv_files[i]);
					strcpy(csv_files[i], csv_files[j]);
					strcpy(csv_files[j], tmp);
				}
			}
		}
		int to_delete = count - 7;
		for (int i = 0; i < to_delete; i++) {
			char fullpath[128];
			snprintf(fullpath, sizeof(fullpath), "/SD:/%.31s", csv_files[i]);
			fs_unlink(fullpath);
			printk("[SD Card] Pruned old log: %s\n", fullpath);
		}
	}
	k_mutex_unlock(&sdcard_fs_mutex);
}

static void sd_logging_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[Thread 4: SD Card Logger] Started (Priority %d)\n", PRIORITY_SD_LOGGER);
	init_sdcard_fat32();

	char current_day_str[24] = "";
	fs_file_t_init(&g_csv_file);
	int64_t last_mount_attempt = k_uptime_get();
	static bool s_ntp_csv_ready = false;

	while (1) {
		telemetry_full_t packet;
		k_msgq_get(&csv_msgq, &packet, K_FOREVER);

		/* Wait until NTP locks real wall-clock time before creating/opening CSV file */
		if (!ntp_synced) {
			continue;
		}

		if (!s_ntp_csv_ready) {
			s_ntp_csv_ready = true;
			printk("[SD Card] NTP locked! Initializing daily CSV telemetry logging...\n");
		}

		if (!sdcard_mounted) {
			int64_t now = k_uptime_get();
			if (now - last_mount_attempt > 10000) {
				last_mount_attempt = now;
				init_sdcard_fat32();
			}
			if (!sdcard_mounted) continue;
		}

		const char *day_str = packet.date_str;

		/* Daily rotation */
		if (strcmp(day_str, current_day_str) != 0) {
			k_mutex_lock(&sdcard_fs_mutex, K_FOREVER);
			if (g_csv_file_is_open) {
				fs_close(&g_csv_file);
				g_csv_file_is_open = false;
			}
			k_mutex_unlock(&sdcard_fs_mutex);

			prune_old_csv_files();

			strncpy(current_day_str, day_str, sizeof(current_day_str));
			char filepath[64];
			snprintf(filepath, sizeof(filepath), "/SD:/telemetry_%s.csv", current_day_str);

			k_mutex_lock(&sdcard_fs_mutex, K_FOREVER);
			struct fs_dirent entry;
			bool exists = (fs_stat(filepath, &entry) == 0);

			int res = fs_open(&g_csv_file, filepath, FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
			if (res == 0) {
				g_csv_file_is_open = true;
				if (!exists || entry.size == 0) {
					const char *hdr = "timestamp_iso,uptime_ms,sample_id,rpm,temp_c,bus_volt_v,curr_ma\n";
					fs_write(&g_csv_file, hdr, strlen(hdr));
					fs_sync(&g_csv_file);
				}
				printk("[SD Card] Opened CSV log: %s\n", filepath);
				k_mutex_lock(&csv_path_mutex, K_FOREVER);
				strncpy(active_csv_filename, filepath, sizeof(active_csv_filename) - 1);
				k_mutex_unlock(&csv_path_mutex);
			}
			k_mutex_unlock(&sdcard_fs_mutex);
		}

		if (g_csv_file_is_open) {
			int32_t t_int = packet.temp_mc / 1000;
			int32_t t_frac = abs(packet.temp_mc % 1000);
			uint32_t v_int = packet.bus_mv / 1000;
			uint32_t v_frac = packet.bus_mv % 1000;

			char line[128];
			int len = snprintf(line, sizeof(line), "%s,%lld,%u,%u,%d.%03d,%u.%03u,%d\n",
			                   packet.iso_time,
			                   (long long)packet.uptime_ms,
			                   packet.sample_id,
			                   packet.motor_rpm,
			                   t_int, t_frac,
			                   v_int, v_frac,
			                   packet.current_ma);

			k_mutex_lock(&sdcard_fs_mutex, K_FOREVER);
			if (g_csv_file_is_open) {
				fs_write(&g_csv_file, line, len);
				if ((packet.sample_id % 10) == 0) {
					fs_sync(&g_csv_file);
				}
			}
			k_mutex_unlock(&sdcard_fs_mutex);
		}
	}
}

/* ========================================================================= */
/* Thread 5: Web Dashboard & SSE Streaming (Priority 8)                      */
/* ========================================================================= */
static const char INDEX_HTML[] =
"<!DOCTYPE html><html><head><meta charset=\"utf-8\"><title>ESP32-C6 Motor Telemetry</title>"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<style>"
"body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;margin:0;padding:12px;background:#121212;color:#e0e0e0;box-sizing:border-box}"
"h1{font-size:1.2rem;margin:0 0 10px 0;display:flex;align-items:center;justify-content:space-between;color:#fff}"
".badge{font-size:0.75rem;padding:3px 8px;border-radius:12px;background:#2e7d32;color:#fff}"
".badge.warn{background:#f57f17}"
".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:8px;margin-bottom:12px}"
".card{background:#1e1e1e;padding:8px 12px;border-radius:6px;border:1px solid #333}"
".card-title{font-size:0.75rem;color:#888;text-transform:uppercase}"
".card-val{font-size:1.3rem;font-weight:700;margin-top:2px;color:#fff}"
".unit{font-size:0.8rem;font-weight:400;color:#aaa}"
".box{background:#1e1e1e;border:1px solid #333;border-radius:6px;padding:10px;margin-bottom:10px}"
".box-title{font-size:0.85rem;font-weight:600;margin-bottom:8px;color:#bbb}"
"canvas{width:100%;height:140px;display:block;background:#181818;border-radius:4px}"
"button{padding:6px 16px;background:#2e7d32;color:#fff;border:none;border-radius:4px;font-weight:600;cursor:pointer}"
"button.stop{background:#c62828}"
"button.dl{background:#1565c0}"
"input[type=number]{background:#2a2a2a;border:1px solid #444;color:#fff;padding:4px 6px;border-radius:4px;box-sizing:border-box;width:100%}"
".label{font-size:0.75rem;color:#888;margin-bottom:3px}"
"#tip{position:fixed;display:none;background:#222;border:1px solid #555;padding:6px 10px;border-radius:4px;font-size:0.75rem;color:#fff;pointer-events:none;z-index:99}"
"</style></head><body>"
"<h1><span>Waveshare ESP32-C6 Motor Engine</span><span id=\"conn\" class=\"badge warn\">Connecting...</span></h1>"
"<div class=\"grid\">"
"<div class=\"card\"><div class=\"card-title\">Motor Speed</div><div class=\"card-val\"><span id=\"v-rpm\">0</span> <span class=\"unit\">RPM</span></div></div>"
"<div class=\"card\"><div class=\"card-title\">Target PWM</div><div class=\"card-val\"><span id=\"v-pwm\">0</span> <span class=\"unit\">%</span></div></div>"
"<div class=\"card\"><div class=\"card-title\">Current &bull; Power</div><div class=\"card-val\"><span id=\"v-curr\">0</span> <span class=\"unit\">mA</span></div></div>"
"<div class=\"card\"><div class=\"card-title\">Bus Voltage</div><div class=\"card-val\"><span id=\"v-volt\">0.00</span> <span class=\"unit\">V</span></div></div>"
"<div class=\"card\"><div class=\"card-title\">Temperature</div><div class=\"card-val\"><span id=\"v-temp\">0.0</span> <span class=\"unit\">&deg;C</span></div></div>"
"<div class=\"card\"><div class=\"card-title\">Active Stage</div><div class=\"card-val\"><span id=\"v-stage\" style=\"font-size:1.0rem\">Idle</span></div></div>"
"</div>"
"<div class=\"box\"><div class=\"box-title\">RPM Speed History</div><canvas id=\"c-rpm\"></canvas></div>"
"<div class=\"box\"><div class=\"box-title\">Current History (mA)</div><canvas id=\"c-curr\"></canvas></div>"
"<div class=\"box\">"
"<div class=\"box-title\">4-Stage PWM Profile Curve (Low &rarr; Ramp Up &rarr; High &rarr; Drop)</div>"
"<canvas id=\"c-curve\" style=\"height:120px;margin-bottom:10px\"></canvas>"
"<div style=\"display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:8px;margin-bottom:10px\">"
"<div><div class=\"label\">1. Low Time (s)</div><input id=\"t-low\" type=\"number\" min=\"0\" max=\"32768\" value=\"410\"></div>"
"<div><div class=\"label\">Target Low PWM (%)</div><input id=\"p-low\" type=\"number\" min=\"0\" max=\"100\" value=\"0\"></div>"
"<div><div class=\"label\">2. Ramp Up Time (s)</div><input id=\"t-up\" type=\"number\" min=\"0\" max=\"32768\" value=\"10\"></div>"
"<div><div class=\"label\">3. High Time (s)</div><input id=\"t-high\" type=\"number\" min=\"0\" max=\"32768\" value=\"170\"></div>"
"<div><div class=\"label\">Target High PWM (%)</div><input id=\"p-high\" type=\"number\" min=\"0\" max=\"100\" value=\"90\"></div>"
"<div><div class=\"label\">4. Drop Time (s)</div><input id=\"t-drop\" type=\"number\" min=\"0\" max=\"32768\" value=\"10\"></div>"
"</div>"
"<div style=\"display:flex;align-items:center;gap:10px;flex-wrap:wrap\">"
"<button onclick=\"startProfile()\">&#x25B6; Run Profile (Loop)</button>"
"<button class=\"stop\" onclick=\"stopProfile()\">&#x23F9; Stop (0%)</button>"
"<a href=\"/api/download/csv\" style=\"text-decoration:none\"><button type=\"button\" class=\"dl\">&#x2193; Download CSV</button></a>"
"<span id=\"prof-status\" style=\"font-size:0.8rem;color:#81c784\">Status: Initializing...</span>"
"</div>"
"</div>"
"<div id=\"tip\"></div>"
"<script>"
"const MAX_PTS=300;let pts=[];"
"const cR=document.getElementById('c-rpm'),xR=cR.getContext('2d');"
"const cC=document.getElementById('c-curr'),xC=cC.getContext('2d');"
"const cP=document.getElementById('c-curve'),xP=cP.getContext('2d');"
"function resize(){[cR,cC,cP].forEach(c=>{c.width=c.clientWidth*window.devicePixelRatio||1;c.height=c.clientHeight*window.devicePixelRatio||1;});render();drawCurve(-1);}"
"window.addEventListener('resize',resize);"
"function drawGrid(ctx,w,h){ctx.strokeStyle='#262626';ctx.lineWidth=1;ctx.beginPath();for(let y=h*0.25;y<h;y+=h*0.25){ctx.moveTo(0,y);ctx.lineTo(w,y);}ctx.stroke();}"
"function render(){"
"const wR=cR.width,hR=cR.height,wC=cC.width,hC=cC.height;"
"xR.clearRect(0,0,wR,hR);xC.clearRect(0,0,wC,hC);"
"drawGrid(xR,wR,hR);drawGrid(xC,wC,hC);"
"if(pts.length<2)return;"
"let maxR=5500;pts.forEach(p=>{if(p.rpm>maxR)maxR=p.rpm;});"
"xR.strokeStyle='#00e676';xR.lineWidth=2;xR.beginPath();"
"pts.forEach((p,i)=>{const x=(i/(pts.length-1))*wR;const y=hR-(p.rpm/maxR)*hR;if(i===0)xR.moveTo(x,y);else xR.lineTo(x,y);});"
"xR.stroke();"
"let maxC=250;pts.forEach(p=>{if(Math.abs(p.curr)>maxC)maxC=Math.abs(p.curr);});"
"xC.strokeStyle='#ff9100';xC.lineWidth=2;xC.beginPath();"
"pts.forEach((p,i)=>{const x=(i/(pts.length-1))*wC;const y=hC-((p.curr+maxC)/(maxC*2))*hC;if(i===0)xC.moveTo(x,y);else xC.lineTo(x,y);});"
"xC.stroke();"
"}"
"function drawCurve(sec){"
"const w=cP.width,h=cP.height;xP.clearRect(0,0,w,h);drawGrid(xP,w,h);"
"const t1=parseInt(document.getElementById('t-low').value)||0;"
"const t2=parseInt(document.getElementById('t-up').value)||0;"
"const t3=parseInt(document.getElementById('t-high').value)||0;"
"const t4=parseInt(document.getElementById('t-drop').value)||0;"
"const pL=parseInt(document.getElementById('p-low').value)||0;"
"const pH=parseInt(document.getElementById('p-high').value)||0;"
"const tot=t1+t2+t3+t4;if(tot<=0)return;"
"const ptsC=[[0,pL],[t1,pL],[t1+t2,pH],[t1+t2+t3,pH],[tot,pL]];"
"xP.strokeStyle='#00e5ff';xP.lineWidth=2;xP.beginPath();"
"ptsC.forEach((pt,i)=>{const x=(pt[0]/tot)*w,y=h-(pt[1]/100)*h;if(i===0)xP.moveTo(x,y);else xP.lineTo(x,y);});"
"xP.stroke();"
"if(sec>=0&&sec<=tot){const curX=(sec/tot)*w;xP.strokeStyle='#fff';xP.lineWidth=2;xP.beginPath();xP.moveTo(curX,0);xP.lineTo(curX,h);xP.stroke();}"
"}"
"function startProfile(){"
"const t1=parseInt(document.getElementById('t-low').value)||0;"
"const t2=parseInt(document.getElementById('t-up').value)||0;"
"const t3=parseInt(document.getElementById('t-high').value)||0;"
"const t4=parseInt(document.getElementById('t-drop').value)||0;"
"const pL=parseInt(document.getElementById('p-low').value)||0;"
"const pH=parseInt(document.getElementById('p-high').value)||0;"
"fetch('/api/profile/start?t_low='+t1+'&p_low='+pL+'&t_up='+t2+'&t_high='+t3+'&p_high='+pH+'&t_drop='+t4).catch(e=>{});"
"}"
"function stopProfile(){fetch('/api/profile/stop').catch(e=>{});}"
"resize();"
"fetch('/api/profile/config').then(r=>r.json()).then(cfg=>{"
"document.getElementById('t-low').value=cfg.t_low;"
"document.getElementById('p-low').value=cfg.p_low;"
"document.getElementById('t-up').value=cfg.t_up;"
"document.getElementById('t-high').value=cfg.t_high;"
"document.getElementById('p-high').value=cfg.p_high;"
"document.getElementById('t-drop').value=cfg.t_drop;"
"drawCurve(-1);"
"}).catch(e=>{});"
"const evt=new EventSource('/events');"
"evt.onopen=()=>{document.getElementById('conn').innerText='Connected';document.getElementById('conn').className='badge';};"
"evt.onerror=()=>{document.getElementById('conn').innerText='Disconnected';document.getElementById('conn').className='badge warn';};"
"evt.onmessage=e=>{"
"try{"
"const d=JSON.parse(e.data);"
"pts.push(d);if(pts.length>MAX_PTS)pts.shift();"
"document.getElementById('v-rpm').innerText=d.rpm;"
"document.getElementById('v-temp').innerText=d.temp;"
"document.getElementById('v-volt').innerText=d.volt;"
"document.getElementById('v-curr').innerText=d.curr;"
"if(d.pwm!==undefined)document.getElementById('v-pwm').innerText=d.pwm;"
"if(d.stage!==undefined){"
"const stNames=['Idle','Low (0%)','Ramp Up','High (90%)','Drop'];"
"document.getElementById('v-stage').innerText=stNames[d.stage]||('Stage '+d.stage);"
"document.getElementById('prof-status').innerText='Loop Active: '+stNames[d.stage]+' ('+d.prof_sec+'s / '+d.prof_tot+'s)';"
"drawCurve(d.prof_sec);"
"}"
"render();"
"}catch(err){}"
"};"
"</script></body></html>";

static int get_query_param_int(const char *req, const char *param, int default_val)
{
	char key[32];
	snprintf(key, sizeof(key), "%s=", param);
	const char *p = strstr(req, key);
	if (!p) return default_val;
	p += strlen(key);
	return atoi(p);
}

/* ========================================================================= */
/* Wi-Fi Management, Diagnostics & Auto-Reconnect Supervisor                */
/* ========================================================================= */
static struct net_mgmt_event_callback wifi_mgmt_cb;
static struct net_mgmt_event_callback ipv4_mgmt_cb;
static uint32_t wifi_attempt_count = 0;
static volatile bool wifi_connecting = false;
static int64_t last_connect_start_ms = 0;

static volatile bool g_target_ap_found = false;
static volatile uint8_t g_target_channel = 0;
static volatile enum wifi_security_type g_target_security = WIFI_SECURITY_TYPE_PSK;
static K_SEM_DEFINE(wifi_scan_done_sem, 0, 1);
static uint32_t scan_ap_count = 0;

static void trigger_wifi_connect(void)
{
	struct net_if *iface = net_if_get_default();
	if (!iface) {
		printk("[Wi-Fi] ERROR: No default network interface found!\n");
		return;
	}

	if (wifi_connected) {
		return;
	}

	/* If a previous connection was in progress, issue DISCONNECT to cleanly reset driver state */
	if (wifi_connecting) {
		printk("[Wi-Fi] Resetting Wi-Fi driver state before initiating fresh connect...\n");
		net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);
		k_msleep(250);
		wifi_connecting = false;
	}

	wifi_attempt_count++;
	last_connect_start_ms = k_uptime_get();
	wifi_connecting = true;

	char mac_str[18] = "Unknown MAC";
	struct net_linkaddr *link_addr = net_if_get_link_addr(iface);
	if (link_addr && link_addr->len == 6) {
		snprintk(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
		         link_addr->addr[0], link_addr->addr[1], link_addr->addr[2],
		         link_addr->addr[3], link_addr->addr[4], link_addr->addr[5]);
	}

#if defined(CONFIG_APP_WIFI_SSID)
	if (strlen(CONFIG_APP_WIFI_SSID) == 0) {
		printk("[Wi-Fi] WARNING: CONFIG_APP_WIFI_SSID is empty! Please configure credentials.conf\n");
		wifi_connecting = false;
		return;
	}
	const char *ssid = CONFIG_APP_WIFI_SSID;
	const char *psk = CONFIG_APP_WIFI_PSK;
	size_t psk_len = strlen(psk);

	uint8_t target_chan = g_target_ap_found ? g_target_channel : WIFI_CHANNEL_ANY;
	enum wifi_security_type sec_type = WIFI_SECURITY_TYPE_NONE;
	const char *sec_str = "Open";

	if (psk_len > 0) {
		if (g_target_ap_found) {
			sec_type = g_target_security;
			sec_str = wifi_security_txt(sec_type);
		} else {
#if defined(CONFIG_ESP32_WIFI_ENABLE_WPA3_SAE)
			/* Alternate between WPA2-PSK (odd attempts) and WPA3-SAE (even attempts) to handle pure WPA3 APs */
			if ((wifi_attempt_count % 2) == 0) {
				sec_type = WIFI_SECURITY_TYPE_SAE_AUTO;
				sec_str = "WPA3-SAE (Auto)";
			} else {
				sec_type = WIFI_SECURITY_TYPE_PSK;
				sec_str = "WPA2-PSK";
			}
#else
			sec_type = WIFI_SECURITY_TYPE_PSK;
			sec_str = "WPA2-PSK";
#endif
		}
	}

	printk("\n[Wi-Fi] -----------------------------------------------------\n");
	printk("[Wi-Fi] Connection Attempt #%u\n", wifi_attempt_count);
	printk("[Wi-Fi]   SSID        : '%s'\n", ssid);
	printk("[Wi-Fi]   Security    : %s\n", sec_str);
	if (target_chan != WIFI_CHANNEL_ANY) {
		printk("[Wi-Fi]   Target CH   : %u (Locked from Site Survey)\n", target_chan);
	} else {
		printk("[Wi-Fi]   Target CH   : All Channels (Scan)\n");
	}
	printk("[Wi-Fi]   Station MAC : %s\n", mac_str);
	printk("[Wi-Fi]   Band        : 2.4 GHz (Wi-Fi 6 / 802.11b/g/n/ax)\n");
	printk("[Wi-Fi] -----------------------------------------------------\n");

	struct wifi_connect_req_params params = {
		.ssid = (const uint8_t *)ssid,
		.ssid_length = strlen(ssid),
		.psk = (const uint8_t *)psk,
		.psk_length = psk_len,
		.channel = target_chan,
		.security = sec_type,
		.band = WIFI_FREQ_BAND_2_4_GHZ,
		.mfp = WIFI_MFP_OPTIONAL,
	};

	int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
	if (ret != 0) {
		wifi_connecting = false;
		printk("[Wi-Fi] ERROR: Connect request rejected immediately by stack (err: %d - %s)\n",
		       ret, ret == -EALREADY ? "Connection in progress" :
		            ret == -EBUSY ? "Device busy" :
		            ret == -EINVAL ? "Invalid parameters" :
		            ret == -ENETDOWN ? "Interface down" : "Check Wi-Fi configuration");
	} else {
		printk("[Wi-Fi] Connect request queued to Wi-Fi driver. Awaiting association...\n");
	}
#else
	printk("[Wi-Fi] WARNING: No SSID configured! Please configure credentials.conf\n");
	wifi_connecting = false;
#endif
}

static bool str_case_eq(const char *s1, const char *s2)
{
	if (!s1 || !s2) return false;
	while (*s1 && *s2) {
		if (tolower((unsigned char)*s1) != tolower((unsigned char)*s2)) {
			return false;
		}
		s1++;
		s2++;
	}
	return (*s1 == '\0' && *s2 == '\0');
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
                                    uint64_t mgmt_event, struct net_if *iface)
{
	switch (mgmt_event) {
	case NET_EVENT_WIFI_SCAN_RESULT: {
		const struct wifi_scan_result *entry = (const struct wifi_scan_result *)cb->info;
		if (!entry || entry->ssid_length == 0) {
			break;
		}
		scan_ap_count++;
		char s_ssid[WIFI_SSID_MAX_LEN + 1];
		size_t slen = entry->ssid_length < WIFI_SSID_MAX_LEN ? entry->ssid_length : WIFI_SSID_MAX_LEN;
		memcpy(s_ssid, entry->ssid, slen);
		s_ssid[slen] = '\0';

		printk("[Wi-Fi Scan]   #%02u: '%s' | Ch %2u | RSSI %3d dBm | %s\n",
		       scan_ap_count, s_ssid, entry->channel, entry->rssi,
		       wifi_security_txt(entry->security));

#if defined(CONFIG_APP_WIFI_SSID)
		if (strcmp(s_ssid, CONFIG_APP_WIFI_SSID) == 0) {
			g_target_ap_found = true;
			g_target_channel = entry->channel;
			g_target_security = entry->security;
			printk("[Wi-Fi Scan]   >>> MATCH FOUND! Targeted AP '%s' on Channel %u\n",
			       s_ssid, entry->channel);
		} else if (str_case_eq(s_ssid, CONFIG_APP_WIFI_SSID)) {
			printk("[Wi-Fi Scan]   >>> WARNING: CASE MISMATCH! Found '%s', but config has '%s'!\n",
			       s_ssid, CONFIG_APP_WIFI_SSID);
		}
#endif
		break;
	}
	case NET_EVENT_WIFI_SCAN_DONE: {
		printk("[Wi-Fi Scan] -----------------------------------------------------\n");
		printk("[Wi-Fi Scan] Survey complete. Total 2.4 GHz APs found: %u\n", scan_ap_count);
#if defined(CONFIG_APP_WIFI_SSID)
		if (!g_target_ap_found) {
			printk("[Wi-Fi Scan] NOTICE: Target SSID '%s' was NOT detected.\n", CONFIG_APP_WIFI_SSID);
			printk("[Wi-Fi Scan] Note: If using an iPhone/Android hotspot, ensure 'Maximize Compatibility' is enabled and the hotspot screen is active.\n");
		}
#endif
		printk("[Wi-Fi Scan] -----------------------------------------------------\n");
		k_sem_give(&wifi_scan_done_sem);
		break;
	}
	case NET_EVENT_WIFI_CONNECT_RESULT: {
		wifi_connecting = false;
		const struct wifi_status *status = (const struct wifi_status *)cb->info;
		if (!status) {
			printk("[Wi-Fi] Event: CONNECT_RESULT (no status data)\n");
			break;
		}
		if (status->status == 0) {
			wifi_connected = true;
			last_dhcp_log = k_uptime_get();
			printk("\n============================================================\n");
			printk("[Wi-Fi] Association SUCCESSFUL!\n");
#if defined(CONFIG_APP_WIFI_SSID)
			printk("[Wi-Fi]   SSID        : %s\n", CONFIG_APP_WIFI_SSID);
#endif
			printk("[Wi-Fi]   Status      : %s (code %d)\n",
			       wifi_conn_status_txt(status->conn_status), status->status);
			printk("[Wi-Fi]   Link State  : UP -> Requesting DHCP IPv4 address...\n");
			printk("============================================================\n");
			net_dhcpv4_start(iface);
		} else {
			wifi_connected = false;
			wifi_has_ip = false;
			wifi_ip_addr[0] = '\0';
			printk("\n============================================================\n");
			printk("[Wi-Fi] Association FAILED!\n");
#if defined(CONFIG_APP_WIFI_SSID)
			printk("[Wi-Fi]   SSID        : %s\n", CONFIG_APP_WIFI_SSID);
#endif
			printk("[Wi-Fi]   Status      : %s (code %d)\n",
			       wifi_conn_status_txt(status->conn_status), status->status);
			if (status->status_code != 0 || status->reason_code != 0) {
				printk("[Wi-Fi]   802.11 Code : Status=%u, Reason=%u\n",
				       status->status_code, status->reason_code);
			}
			printk("[Wi-Fi]   Action      : Auto-reconnect supervisor will retry in 10s...\n");
			printk("============================================================\n");
		}
		break;
	}
	case NET_EVENT_WIFI_DISCONNECT_RESULT: {
		wifi_connected = false;
		wifi_connecting = false;
		wifi_has_ip = false;
		wifi_ip_addr[0] = '\0';
		const struct wifi_status *status = (const struct wifi_status *)cb->info;
		printk("\n============================================================\n");
		printk("[Wi-Fi] Disconnected from AP!\n");
		if (status) {
			printk("[Wi-Fi]   Reason      : %s (code %d)\n",
			       wifi_disconn_reason_txt(status->disconn_reason), status->status);
			if (status->reason_code != 0) {
				printk("[Wi-Fi]   802.11 Reason: %u\n", status->reason_code);
			}
		}
		printk("[Wi-Fi]   Action      : Network down, auto-reconnect supervisor active...\n");
		printk("============================================================\n");
		break;
	}
	case NET_EVENT_IPV4_DHCP_START: {
		printk("[DHCP] Client active: sending DHCP Discover to router...\n");
		break;
	}
	case NET_EVENT_IPV4_DHCP_BOUND: {
		printk("[DHCP] Offer accepted: DHCPv4 lease acquired successfully!\n");
		break;
	}
	case NET_EVENT_IPV4_DHCP_STOP: {
		printk("[DHCP] Client stopped.\n");
		break;
	}
	case NET_EVENT_IPV4_ADDR_ADD: {
		struct net_if_config *cfg = net_if_get_config(iface);
		if (!cfg || !cfg->ip.ipv4) {
			break;
		}
		for (int i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
			if (cfg->ip.ipv4->unicast[i].ipv4.addr_type == NET_ADDR_DHCP ||
			    cfg->ip.ipv4->unicast[i].ipv4.is_used) {
				char ip_str[INET_ADDRSTRLEN];
				net_addr_ntop(AF_INET,
				              &cfg->ip.ipv4->unicast[i].ipv4.address.in_addr,
				              ip_str, sizeof(ip_str));
				strncpy(wifi_ip_addr, ip_str, sizeof(wifi_ip_addr) - 1);
				wifi_has_ip = true;

				char nm_str[INET_ADDRSTRLEN] = "0.0.0.0";
				char gw_str[INET_ADDRSTRLEN] = "0.0.0.0";
				net_addr_ntop(AF_INET, &cfg->ip.ipv4->unicast[i].netmask,
				              nm_str, sizeof(nm_str));
				net_addr_ntop(AF_INET, &cfg->ip.ipv4->gw,
				              gw_str, sizeof(gw_str));

				printk("\n=================================================================\n");
				printk(">>> ESP32-C6-LCD-1.47 Web Dashboard is LIVE! <<<\n");
				printk("=================================================================\n");
				printk("  [IP Address]   : %s\n", ip_str);
				printk("  [Subnet Mask]  : %s\n", nm_str);
				printk("  [Gateway]      : %s\n", gw_str);
				printk("  [Web URL]      : http://%s/\n", ip_str);
				printk("  [SSE Stream]   : http://%s/events\n", ip_str);
				printk("  [Download CSV] : http://%s/api/download/csv\n", ip_str);
				printk("=================================================================\n\n");
				k_sem_give(&ntp_trigger_sem);
				break;
			}
		}
		break;
	}
	case NET_EVENT_IPV4_ADDR_DEL: {
		printk("[Wi-Fi] IPv4 address lease released / expired (%s)\n", wifi_ip_addr);
		wifi_ip_addr[0] = '\0';
		wifi_has_ip = false;
		break;
	}
	default:
		break;
	}
}

static void wifi_http_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[Thread 5: HTTP Server & Wi-Fi] Started (Priority %d)\n", PRIORITY_WIFI_HTTP);

	struct net_if *iface = net_if_get_default();

	net_mgmt_init_event_callback(&wifi_mgmt_cb, wifi_mgmt_event_handler,
	                             NET_EVENT_WIFI_CONNECT_RESULT |
	                             NET_EVENT_WIFI_DISCONNECT_RESULT |
	                             NET_EVENT_WIFI_SCAN_RESULT |
	                             NET_EVENT_WIFI_SCAN_DONE);
	net_mgmt_add_event_callback(&wifi_mgmt_cb);

	net_mgmt_init_event_callback(&ipv4_mgmt_cb, wifi_mgmt_event_handler,
	                             NET_EVENT_IPV4_ADDR_ADD | NET_EVENT_IPV4_ADDR_DEL |
	                             NET_EVENT_IPV4_DHCP_START | NET_EVENT_IPV4_DHCP_BOUND |
	                             NET_EVENT_IPV4_DHCP_STOP);
	net_mgmt_add_event_callback(&ipv4_mgmt_cb);

	for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
		sse_clients[i].fd = -1;
		sse_clients[i].last_sent_id = 0;
	}

	int server_fd = -1;

	/* Run active 2.4 GHz site survey scan to verify spectrum & target channel */
	if (iface) {
		printk("\n[Wi-Fi Scan] Running 2.4 GHz site survey to locate '%s'...\n", CONFIG_APP_WIFI_SSID);
		scan_ap_count = 0;
		struct wifi_scan_params scan_params = {
			.scan_type = WIFI_SCAN_TYPE_ACTIVE,
			.bands = WIFI_FREQ_BAND_2_4_GHZ,
		};
		int s_rc = net_mgmt(NET_REQUEST_WIFI_SCAN, iface, &scan_params, sizeof(scan_params));
		if (s_rc == 0) {
			k_sem_take(&wifi_scan_done_sem, K_SECONDS(5));
		}
	}

	/* Trigger initial connection attempt */
	trigger_wifi_connect();

	while (1) {
		int64_t now = k_uptime_get();

		/* Auto-reconnect supervisor: non-blocking, guarded against overlapping connection attempts */
		if (!wifi_connected) {
			if (wifi_connecting) {
				/* Watchdog: If in-flight connection has been waiting > 20s without driver callback, reset and retry */
				if (now - last_connect_start_ms > 20000) {
					printk("[Wi-Fi] Connection attempt timed out (20s). Retrying with clean state...\n");
					trigger_wifi_connect();
				}
			} else {
				/* If disconnected and not connecting, wait 10s between attempts */
				if (now - last_connect_start_ms > 10000) {
					trigger_wifi_connect();
				}
			}
		}

		/* If associated but waiting for DHCP IPv4 address */
		if (!wifi_has_ip) {
			if (wifi_connected && (now - last_dhcp_log > 10000)) {
				last_dhcp_log = now;
#if defined(CONFIG_APP_WIFI_SSID)
				printk("[Wi-Fi] Associated with AP '%s'. Waiting for DHCP IPv4 lease from router...\n",
				       CONFIG_APP_WIFI_SSID);
#endif
			}
			/* Clean up sockets if disconnected */
			for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
				if (sse_clients[i].fd >= 0) {
					close(sse_clients[i].fd);
					sse_clients[i].fd = -1;
				}
			}
			if (server_fd >= 0) {
				close(server_fd);
				server_fd = -1;
			}
			k_sleep(K_MSEC(250));
			continue;
		}

		/* Initialize listening server socket once IP is acquired */
		if (server_fd < 0) {
			server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
			if (server_fd < 0) {
				printk("[HTTP Server] Socket creation failed\n");
				k_sleep(K_MSEC(1000));
				continue;
			}

			int opt = 1;
			setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

			struct sockaddr_in bind_addr = {
				.sin_family = AF_INET,
				.sin_port = htons(HTTP_PORT),
				.sin_addr.s_addr = htonl(INADDR_ANY),
			};

			if (bind(server_fd, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
				printk("[HTTP Server] Bind failed on port %d, retrying in 1s...\n", HTTP_PORT);
				close(server_fd);
				server_fd = -1;
				k_sleep(K_MSEC(1000));
				continue;
			}

			listen(server_fd, 5);
			printk("[HTTP Server] Listening on http://%s:%d/\n", wifi_ip_addr, HTTP_PORT);
		}

		struct zsock_pollfd pfd[1 + MAX_SSE_CLIENTS];
		int pfd_to_client[MAX_SSE_CLIENTS];
		int num_fds = 0;

		pfd[0].fd = server_fd;
		pfd[0].events = ZSOCK_POLLIN;
		pfd[0].revents = 0;
		num_fds = 1;

		for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
			if (sse_clients[i].fd >= 0) {
				pfd[num_fds].fd = sse_clients[i].fd;
				pfd[num_fds].events = ZSOCK_POLLIN | ZSOCK_POLLERR | ZSOCK_POLLHUP;
				pfd[num_fds].revents = 0;
				pfd_to_client[num_fds - 1] = i;
				num_fds++;
			}
		}

		int poll_ret = poll(pfd, num_fds, 100);

		if (poll_ret > 0 && (pfd[0].revents & (ZSOCK_POLLERR | ZSOCK_POLLHUP))) {
			printk("[HTTP Server] Server listening socket error, recycling...\n");
			close(server_fd);
			server_fd = -1;
			break;
		}

		if (poll_ret > 0 && (pfd[0].revents & ZSOCK_POLLIN)) {
			struct sockaddr_in client_addr;
			socklen_t client_len = sizeof(client_addr);
			int client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len);

			if (client_fd >= 0) {
				char rx_buf[512] = {0};
				recv(client_fd, rx_buf, sizeof(rx_buf) - 1, 0);

				if (strncmp(rx_buf, "GET /events", 11) == 0) {
					const char *hdr = "HTTP/1.1 200 OK\r\n"
					                  "Content-Type: text/event-stream\r\n"
					                  "Cache-Control: no-cache\r\n"
					                  "Connection: keep-alive\r\n"
					                  "Access-Control-Allow-Origin: *\r\n\r\n";
					send_all(client_fd, hdr, strlen(hdr));

					int slot = -1;
					for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
						if (sse_clients[i].fd < 0) { slot = i; break; }
					}
					if (slot >= 0) {
						sse_clients[slot].fd = client_fd;
						sse_clients[slot].last_sent_id = 0;
					} else {
						close(client_fd);
					}
				} else if (strncmp(rx_buf, "GET /api/download/csv", 21) == 0) {
					if (g_download_in_progress) {
						const char *busy = "HTTP/1.1 503 Service Unavailable\r\nContent-Length: 17\r\n\r\nDownload in use\r\n";
						send_all(client_fd, busy, strlen(busy));
						close(client_fd);
					} else {
						g_download_client_fd = client_fd;
						g_download_in_progress = true;
						k_sem_give(&download_start_sem);
					}
				} else if (strncmp(rx_buf, "GET /api/profile/config", 23) == 0) {
					k_spinlock_key_t pkey = k_spin_lock(&profile_spinlock);
					pwm_profile_ctrl_t cfg = g_prof_cfg;
					k_spin_unlock(&profile_spinlock, pkey);

					char body[128];
					int blen = snprintf(body, sizeof(body),
					                    "{\"t_low\":%u,\"p_low\":%u,\"t_up\":%u,\"t_high\":%u,\"p_high\":%u,\"t_drop\":%u}\r\n",
					                    cfg.t_low_sec, cfg.p_low_pct, cfg.t_up_sec, cfg.t_high_sec, cfg.p_high_pct, cfg.t_drop_sec);
					char hdr[128];
					int hlen = snprintf(hdr, sizeof(hdr),
					                    "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: %d\r\nConnection: close\r\n\r\n", blen);
					send_all(client_fd, hdr, hlen);
					send_all(client_fd, body, blen);
					close(client_fd);
				} else if (strncmp(rx_buf, "GET /api/profile/start", 22) == 0) {
					k_spinlock_key_t pkey = k_spin_lock(&profile_spinlock);
					if (strstr(rx_buf, "t_low=")) {
						g_prof_cfg.t_low_sec  = get_query_param_int(rx_buf, "t_low", g_prof_cfg.t_low_sec);
						g_prof_cfg.p_low_pct  = get_query_param_int(rx_buf, "p_low", g_prof_cfg.p_low_pct);
						g_prof_cfg.t_up_sec   = get_query_param_int(rx_buf, "t_up", g_prof_cfg.t_up_sec);
						g_prof_cfg.t_high_sec = get_query_param_int(rx_buf, "t_high", g_prof_cfg.t_high_sec);
						g_prof_cfg.p_high_pct = get_query_param_int(rx_buf, "p_high", g_prof_cfg.p_high_pct);
						g_prof_cfg.t_drop_sec = get_query_param_int(rx_buf, "t_drop", g_prof_cfg.t_drop_sec);
					}
					g_prof_cfg.action = PWM_ACTION_START_PROFILE;
					k_spin_unlock(&profile_spinlock, pkey);

					const char *res = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 17\r\nConnection: close\r\n\r\n{\"status\":\"started\"}\r\n";
					send_all(client_fd, res, strlen(res));
					close(client_fd);
				} else if (strncmp(rx_buf, "GET /api/profile/stop", 21) == 0) {
					k_spinlock_key_t pkey = k_spin_lock(&profile_spinlock);
					g_prof_cfg.action = PWM_ACTION_STOP_PROFILE;
					k_spin_unlock(&profile_spinlock, pkey);

					const char *res = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 17\r\nConnection: close\r\n\r\n{\"status\":\"stopped\"}\r\n";
					send_all(client_fd, res, strlen(res));
					close(client_fd);
				} else {
					/* Serve main dashboard HTML */
					char hdr[128];
					int hlen = snprintf(hdr, sizeof(hdr),
					                    "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: %d\r\nConnection: close\r\n\r\n", (int)sizeof(INDEX_HTML) - 1);
					send_all(client_fd, hdr, hlen);
					send_all(client_fd, INDEX_HTML, sizeof(INDEX_HTML) - 1);
					close(client_fd);
				}
			}
		}

		/* Check for SSE client disconnects */
		if (poll_ret > 0) {
			for (int k = 1; k < num_fds; k++) {
				int c_idx = pfd_to_client[k - 1];
				int cfd = sse_clients[c_idx].fd;
				if (cfd < 0) continue;

				if (pfd[k].revents & (ZSOCK_POLLERR | ZSOCK_POLLHUP)) {
					close(cfd);
					sse_clients[c_idx].fd = -1;
				} else if (pfd[k].revents & ZSOCK_POLLIN) {
					char dummy[16];
					int r = recv(cfd, dummy, sizeof(dummy), ZSOCK_MSG_DONTWAIT);
					if (r <= 0) {
						if (r == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
							close(cfd);
							sse_clients[c_idx].fd = -1;
						}
					}
				}
			}
		}

		/* Push SSE frame */
		telemetry_full_t cur;
		bool has_pkt = false;
		k_spinlock_key_t key = k_spin_lock(&telemetry_spinlock);
		if (has_telemetry) {
			cur = latest_telemetry;
			has_pkt = true;
		}
		k_spin_unlock(&telemetry_spinlock, key);

		if (has_pkt) {
			char sse_data[320];
			int sse_len = snprintf(sse_data, sizeof(sse_data),
			                       "data: {\"id\":%u,\"rpm\":%u,\"pwm\":%u,\"stage\":%u,\"prof_sec\":%u,\"prof_tot\":%u,\"temp\":%d.%d,\"volt\":%u.%02u,\"curr\":%d,\"uptime\":%lld,\"time\":\"%s\"}\n\n",
			                       cur.sample_id, cur.motor_rpm, cur.pwm_pct, cur.profile_stage,
			                       cur.profile_elapsed_sec, cur.profile_total_sec,
			                       cur.temp_mc / 1000, abs(cur.temp_mc % 1000) / 100,
			                       cur.bus_mv / 1000, (cur.bus_mv % 1000) / 10,
			                       cur.current_ma, (long long)cur.uptime_ms, cur.iso_time);

			for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
				if (sse_clients[i].fd >= 0 && sse_clients[i].last_sent_id != cur.sample_id) {
					if (send_all(sse_clients[i].fd, sse_data, sse_len) < 0) {
						close(sse_clients[i].fd);
						sse_clients[i].fd = -1;
					} else {
						sse_clients[i].last_sent_id = cur.sample_id;
					}
				}
			}
		}
	}
}

/* ========================================================================= */
/* Thread 6: Non-blocking Background CSV Download (Priority 9)               */
/* ========================================================================= */
static void csv_download_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[Thread 6: CSV Downloader] Started (Priority %d)\n", PRIORITY_DOWNLOAD);

	while (1) {
		k_sem_take(&download_start_sem, K_FOREVER);
		int client_fd = g_download_client_fd;
		if (client_fd < 0) continue;

		char target_file[64];
		k_mutex_lock(&csv_path_mutex, K_FOREVER);
		strncpy(target_file, active_csv_filename, sizeof(target_file) - 1);
		k_mutex_unlock(&csv_path_mutex);

		if (strlen(target_file) == 0) {
			const char *nf = "HTTP/1.1 404 Not Found\r\nContent-Length: 14\r\n\r\nNo CSV Active\r\n";
			send_all(client_fd, nf, strlen(nf));
			close(client_fd);
			g_download_in_progress = false;
			continue;
		}

		k_mutex_lock(&sdcard_fs_mutex, K_FOREVER);
		struct fs_file_t rfile;
		fs_file_t_init(&rfile);
		int res = fs_open(&rfile, target_file, FS_O_READ);
		k_mutex_unlock(&sdcard_fs_mutex);

		if (res != 0) {
			const char *err = "HTTP/1.1 500 Internal Error\r\nContent-Length: 16\r\n\r\nCannot Open Log\r\n";
			send_all(client_fd, err, strlen(err));
			close(client_fd);
			g_download_in_progress = false;
			continue;
		}

		const char *fname = strrchr(target_file, '/');
		fname = fname ? fname + 1 : target_file;

		char header[256];
		int hlen = snprintf(header, sizeof(header),
		                    "HTTP/1.1 200 OK\r\n"
		                    "Content-Type: text/csv\r\n"
		                    "Content-Disposition: attachment; filename=\"%s\"\r\n"
		                    "Connection: close\r\n\r\n", fname);
		send_all(client_fd, header, hlen);

		char read_buf[512];
		while (1) {
			k_mutex_lock(&sdcard_fs_mutex, K_FOREVER);
			ssize_t bytes_read = fs_read(&rfile, read_buf, sizeof(read_buf));
			k_mutex_unlock(&sdcard_fs_mutex);

			if (bytes_read <= 0) break;
			if (send_all(client_fd, read_buf, (size_t)bytes_read) < 0) break;
			k_msleep(2); /* Throttle to avoid starving other threads */
		}

		k_mutex_lock(&sdcard_fs_mutex, K_FOREVER);
		fs_close(&rfile);
		k_mutex_unlock(&sdcard_fs_mutex);

		close(client_fd);
		g_download_in_progress = false;
	}
}

/* ========================================================================= */
/* Thread 7: Decoupled NTP Client (Priority 10)                              */
/* ========================================================================= */
static void ntp_sync_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[Thread 7: NTP Client] Started (Priority %d)\n", PRIORITY_NTP);

	while (1) {
		k_sem_take(&ntp_trigger_sem, K_FOREVER);
		printk("[NTP] Querying pool.ntp.org...\n");

		struct sntp_time sntp_time;
		int ret = sntp_simple("pool.ntp.org", 5000, &sntp_time);
		if (ret == 0) {
			k_spinlock_key_t key = k_spin_lock(&time_spinlock);
			ntp_epoch_sec = sntp_time.seconds;
			ntp_sync_uptime = k_uptime_get();
			ntp_synced = true;
			k_spin_unlock(&time_spinlock, key);

			char iso[32], date[16];
			get_formatted_time(iso, sizeof(iso), date, sizeof(date));
			printk("[NTP] Synchronized! Local (UTC+8): %s (epoch %lld)\n",
			       iso, (long long)sntp_time.seconds);
		} else {
			printk("[NTP] Sync failed (%d), retrying in 30s\n", ret);
			k_sleep(K_SECONDS(30));
			if (wifi_has_ip) k_sem_give(&ntp_trigger_sem);
			continue;
		}

		k_sleep(K_SECONDS(3600)); /* Periodic 1-hour sync */
		if (wifi_has_ip) k_sem_give(&ntp_trigger_sem);
	}
}

/* ========================================================================= */
/* Thread Definitions                                                        */
/* ========================================================================= */
K_THREAD_DEFINE(thread_motor_speed, STACK_MOTOR_SPEED, motor_speed_thread_fn,
                NULL, NULL, NULL, PRIORITY_MOTOR_SPEED, 0, 0);

K_THREAD_DEFINE(thread_sensor, STACK_SENSOR, sensor_aggregator_thread_fn,
                NULL, NULL, NULL, PRIORITY_SENSOR, 0, 0);

K_THREAD_DEFINE(thread_display, STACK_DISPLAY, display_thread_fn,
                NULL, NULL, NULL, PRIORITY_DISPLAY, 0, 0);

K_THREAD_DEFINE(thread_sd_logger, STACK_SD_LOGGER, sd_logging_thread_fn,
                NULL, NULL, NULL, PRIORITY_SD_LOGGER, 0, 0);

K_THREAD_DEFINE(thread_wifi_http, STACK_WIFI_HTTP, wifi_http_thread_fn,
                NULL, NULL, NULL, PRIORITY_WIFI_HTTP, 0, 0);

K_THREAD_DEFINE(thread_csv_download, STACK_DOWNLOAD, csv_download_thread_fn,
                NULL, NULL, NULL, PRIORITY_DOWNLOAD, 0, 0);

K_THREAD_DEFINE(thread_ntp, STACK_NTP, ntp_sync_thread_fn,
                NULL, NULL, NULL, PRIORITY_NTP, 0, 0);

/* ========================================================================= */
/* Main Entry Point                                                          */
/* ========================================================================= */
int main(void)
{
	printk("=================================================================\n");
	printk("Waveshare ESP32-C6-LCD-1.47 Unified Real-Time Motor Engine\n");
	printk("Target: esp32c6_lcd_1_47/esp32c6/hpcore (RISC-V 160MHz)\n");
	printk("=================================================================\n");

	const struct device *const timer_dev = DEVICE_DT_GET(TIMER_NODE);
	if (!timer_dev || !device_is_ready(timer_dev)) {
		printk("[MAIN] Timer counter device not ready!\n");
	} else {
		struct counter_top_cfg top_cfg = {
			.ticks = counter_us_to_ticks(timer_dev, TIMER_INTERVAL_US),
			.callback = timer_top_subroutine,
			.user_data = NULL,
			.flags = 0,
		};
		counter_set_top_value(timer_dev, &top_cfg);
		counter_start(timer_dev);
		printk("[MAIN] Periodic counter started at %d Hz\n", CONFIG_APP_SAMPLE_RATE_HZ);
	}

	while (1) {
		k_sleep(K_SECONDS(30));
		printk("[Supervisor] Uptime: %lld s | IP: %s | Wi-Fi: %s | NTP: %s\n",
		       (long long)(k_uptime_get() / 1000),
		       wifi_has_ip ? wifi_ip_addr : "0.0.0.0",
		       wifi_connected ? "Connected" : "Disconnected",
		       ntp_synced ? "Synced" : "Pending");
	}
	return 0;
}
