/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Waveshare ESP32-S3-GEEK PRO_CPU (Core 0): Primary Host Application.
 *
 * Multi-Core AMP Architecture (Sysbuild):
 *   [APP_CPU (Core 1): Motor & Sensor Engine]
 *               │ (IPM Channel 2: telemetry_packet_t)
 *               │ (IPM Channel 3: ipc_print_packet_t)
 *               ▼
 *   [PRO_CPU (Core 0): IPM ISR / Callback]
 *        ├── (ipc_print_msgq) ──> [Thread 0: Core 1 IPC Print Console (Priority 6)]
 *        ├── (csv_msgq)       ──> [Thread 1: FAT32 SDCard CSV Logger (Priority 7)]
 *        └── (spinlock)       ──> [Thread 2: Wi-Fi / SSE Web Server (Priority 8)]
 *                                          │ (signals on IP acquired)
 *                                          ▼
 *                                   [Thread 3: Background NTP Client (Priority 10)]
 *
 * Features:
 *   1. Inter-Processor Mailbox (IPM) zero-jitter telemetry ingestion from Core 1.
 *   2. FAT32 SD Card Support with auto-format on startup if unformatted.
 *   3. Daily auto-rotating CSV file logging: /SD:/telemetry_YYYY-MM-DD.csv
 *   4. NTP client synchronization for absolute wall-clock timestamps in CSV & Web UI.
 *   5. Multi-client SSE server (/events) real-time broadcast stream with zero blocking.
 *   6. Zero-dependency Canvas Dashboard with interactive hover tooltip & crosshair inspect.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <zephyr/fs/fs.h>
#include <ff.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/sntp.h>

#include "telemetry.h"

/* POSIX socket aliases */
#define socket(f, t, p)             zsock_socket(f, t, p)
#define bind(s, a, l)               zsock_bind(s, a, l)
#define listen(s, b)                zsock_listen(s, b)
#define accept(s, a, l)             zsock_accept(s, a, l)
#define send(s, b, l, f)            zsock_send(s, b, l, f)
#define recv(s, b, l, f)            zsock_recv(s, b, l, f)
#define poll(p, n, t)               zsock_poll(p, n, t)
#define setsockopt(s, l, o, v, len) zsock_setsockopt(s, l, o, v, len)
#define close(s)                    zsock_close(s)

/* ========================================================================= */
/* Configuration Constants                                                   */
/* ========================================================================= */
#define HTTP_PORT                   80
#define MAX_SSE_CLIENTS             4

#ifndef MAX_CSV_FILES_RETAINED
#define MAX_CSV_FILES_RETAINED      7     /* Keep up to 7 daily logs, prune oldest */
#endif

#ifndef CONFIG_APP_SAMPLE_RATE_HZ
#define CONFIG_APP_SAMPLE_RATE_HZ   10    /* 10 Hz sampling & streaming */
#endif

#define PRIORITY_IPC_CONSOLE        6     /* Thread 0: Core 1 IPC Print Console */
#define PRIORITY_LOW                7     /* Thread 1: SD Card CSV Logger */
#define PRIORITY_WIFI               8     /* Thread 2: Wi-Fi & Web Server */
#define PRIORITY_DOWNLOAD           9     /* Thread 4: Background CSV Downloader */
#define PRIORITY_NTP                10    /* Thread 3: Background NTP Client */

#define IPC_CONSOLE_STACK_SIZE      2048
#define CSV_STACK_SIZE              8192
#define WIFI_STACK_SIZE             6144
#define DOWNLOAD_STACK_SIZE         4096
#define NTP_STACK_SIZE              3072

/* Devicetree IPM node */
static const struct device *ipm_dev;

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
	char     iso_time[32];         /* "YYYY-MM-DDTHH:MM:SSZ"              */
	char     date_str[16];         /* "YYYY-MM-DD"                        */
} telemetry_full_t;

/* Active 4-stage PWM Profile Configuration */
static ipm_pwm_ctrl_t g_active_profile_cfg = {
	.t_low_sec = DEFAULT_PROF_T_LOW,
	.t_up_sec = DEFAULT_PROF_T_UP,
	.t_high_sec = DEFAULT_PROF_T_HIGH,
	.t_drop_sec = DEFAULT_PROF_T_DROP,
	.p_low_pct = DEFAULT_PROF_P_LOW,
	.p_high_pct = DEFAULT_PROF_P_HIGH,
	.action = PWM_ACTION_START_PROFILE,
	.manual_pwm = 0,
};

typedef struct {
	int fd;
	uint32_t last_sent_id;
} sse_client_t;

/* Message Queues */
K_MSGQ_DEFINE(csv_msgq, sizeof(telemetry_full_t), 8, 4);
K_MSGQ_DEFINE(ipc_print_msgq, sizeof(ipc_print_packet_t), 16, 4);

/* Synchronization between IPM and SSE server */
static struct k_spinlock telemetry_spinlock;
static telemetry_full_t latest_telemetry;
static bool has_telemetry = false;

/* Active CSV Path shared between logger & SSE replay */
static K_MUTEX_DEFINE(csv_path_mutex);
static char active_csv_filename[64] = "";

/* Real-Time Clock / NTP State */
static struct k_spinlock time_spinlock;
static int64_t ntp_epoch_sec = 0;
static int64_t ntp_sync_uptime = 0;
static bool ntp_synced = false;
static K_SEM_DEFINE(ntp_trigger_sem, 0, 1);

/* Wi-Fi & Network State */
static volatile bool wifi_connected = false;
static volatile bool wifi_connecting = false;
static volatile bool wifi_has_ip = false;
static char assigned_ipv4_str[NET_IPV4_ADDR_LEN] = "0.0.0.0";
static uint32_t wifi_attempt_count = 0;
static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;

/* FatFS File System mount descriptor & Global File Mutex */
static FATFS fat_fs;
static struct fs_mount_t fs_mnt = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
	.mnt_point = "/SD:",
};
static bool sdcard_mounted = false;
static K_MUTEX_DEFINE(sdcard_fs_mutex);
static struct fs_file_t g_csv_file;
static bool g_csv_file_is_open = false;

static void sdcard_flush_sync(void)
{
	k_mutex_lock(&sdcard_fs_mutex, K_FOREVER);
	if (g_csv_file_is_open) {
		fs_sync(&g_csv_file);
	}
	k_mutex_unlock(&sdcard_fs_mutex);
}

/* ========================================================================= */
/* NTP & Time Formatting Helpers (UTC+8 Default)                             */
/* ========================================================================= */
#ifndef CONFIG_APP_TIMEZONE_OFFSET_SEC
#define CONFIG_APP_TIMEZONE_OFFSET_SEC  (8 * 3600)  /* UTC+8 (28,800 seconds) */
#endif

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

static void sync_time_to_app_cpu(int64_t now_ms, bool force)
{
	static int64_t last_sync_ms = 0;
	if (!force && (now_ms - last_sync_ms < 250)) {
		return;
	}
	last_sync_ms = now_ms;

	if (!ipm_dev || !device_is_ready(ipm_dev)) {
		return;
	}

	int64_t now_sec = 0;
	int64_t ms_from_sync = 0;
	k_spinlock_key_t key = k_spin_lock(&time_spinlock);
	bool is_synced = ntp_synced;
	if (is_synced) {
		ms_from_sync = now_ms - ntp_sync_uptime;
		now_sec = ntp_epoch_sec + (ms_from_sync / 1000);
	}
	k_spin_unlock(&time_spinlock, key);

	ipm_time_sync_t t_sync;
	if (is_synced) {
		time_t t = (time_t)(now_sec + CONFIG_APP_TIMEZONE_OFFSET_SEC);
		struct tm tm_val;
		gmtime_r(&t, &tm_val);

		uint32_t sec_of_day = (uint32_t)tm_val.tm_hour * 3600U +
		                      (uint32_t)tm_val.tm_min * 60U +
		                      (uint32_t)tm_val.tm_sec;
		uint16_t ms_part = (uint16_t)(((ms_from_sync % 1000LL) + 1000LL) % 1000LL);

		t_sync.month = (uint8_t)(tm_val.tm_mon + 1);
		t_sync.day = (uint8_t)tm_val.tm_mday;
		t_sync.hour = (uint8_t)tm_val.tm_hour;
		t_sync.min = (uint8_t)tm_val.tm_min;
		t_sync.sec = (uint8_t)tm_val.tm_sec;
		t_sync.synced = 1;
		t_sync.ms_part = ms_part;
		t_sync.sec_of_day = sec_of_day;
	} else {
		uint32_t uptime_sec = (uint32_t)(now_ms / 1000ULL);
		t_sync.month = 0;
		t_sync.day = 0;
		t_sync.hour = (uint8_t)((uptime_sec / 3600U) % 24U);
		t_sync.min = (uint8_t)((uptime_sec % 3600U) / 60U);
		t_sync.sec = (uint8_t)(uptime_sec % 60U);
		t_sync.synced = 0;
		t_sync.ms_part = (uint16_t)(now_ms % 1000ULL);
		t_sync.sec_of_day = uptime_sec % 86400U;
	}

	ipm_send(ipm_dev, 0, IPM_CHANNEL_TIME_SYNC, &t_sync, sizeof(t_sync));
}

/* Helper to reliably send all bytes across a TCP socket with non-blocking retry */
static int send_all(int sock, const void *buf, size_t len)
{
	const char *p = (const char *)buf;
	while (len > 0) {
		int sent = send(sock, p, len, 0);
		if (sent <= 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				k_msleep(1);
				continue;
			}
			return -1;
		}
		p += sent;
		len -= sent;
	}
	return 0;
}

/* Non-blocking SSE pusher: skips frame on transient backpressure without dropping client */
static int send_sse_frame(int sock, const void *buf, size_t len)
{
	int sent = send(sock, buf, len, ZSOCK_MSG_DONTWAIT);
	if (sent < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS) {
			return 0; /* Transient full queue, skip without disconnecting */
		}
		return -1; /* Fatal socket disconnect */
	}
	return 1;
}

/* ========================================================================= */
/* Inter-Processor Mailbox (IPM) Callback from APP_CPU                       */
/* ========================================================================= */
static void ipm_receive_callback(const struct device *dev, void *user_data,
                                 uint32_t id, volatile void *data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (id == IPM_CHANNEL_APP_TELEMETRY && data != NULL) {
		telemetry_packet_t raw_pkt;
		memcpy(&raw_pkt, (const void *)data, sizeof(raw_pkt));

		telemetry_full_t full_pkt;
		full_pkt.sample_id = raw_pkt.sample_id;
		full_pkt.motor_rpm = raw_pkt.motor_rpm;
		full_pkt.temp_mc = raw_pkt.temp_mc;
		full_pkt.bus_mv = raw_pkt.bus_mv;
		full_pkt.current_ma = raw_pkt.current_ma;
		full_pkt.uptime_ms = raw_pkt.uptime_ms;
		full_pkt.pwm_pct = raw_pkt.pwm_pct;
		full_pkt.profile_stage = raw_pkt.profile_stage;
		full_pkt.profile_elapsed_sec = raw_pkt.profile_elapsed_sec;
		full_pkt.profile_total_sec = raw_pkt.profile_total_sec;

		get_formatted_time(full_pkt.iso_time, sizeof(full_pkt.iso_time),
		                   full_pkt.date_str, sizeof(full_pkt.date_str));

		/* Enqueue to SD Card logger only after real wall-clock time is locked by NTP */
		if (ntp_synced) {
			k_msgq_put(&csv_msgq, &full_pkt, K_NO_WAIT);
		}

		/* Update latest telemetry for SSE clients */
		k_spinlock_key_t key = k_spin_lock(&telemetry_spinlock);
		latest_telemetry = full_pkt;
		has_telemetry = true;
		k_spin_unlock(&telemetry_spinlock, key);
	} else if (id == IPM_CHANNEL_APP_PRINT && data != NULL) {
		ipc_print_packet_t print_pkt;
		strncpy(print_pkt.msg, (const char *)data, sizeof(print_pkt.msg) - 1);
		print_pkt.msg[sizeof(print_pkt.msg) - 1] = '\0';
		k_msgq_put(&ipc_print_msgq, &print_pkt, K_NO_WAIT);
	}
}

/* ========================================================================= */
/* Embedded HTML5 / JS Dashboard with Interactive Hover Tooltip & Crosshairs */
/* ========================================================================= */
static const char HTTP_HTML_PAGE[] =
"<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
"<title>ESP32-S3-GEEK Live Telemetry (AMP)</title>"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<style>"
"body{font-family:system-ui,-apple-system,sans-serif;background:#121212;color:#e0e0e0;margin:0;padding:16px}"
"h1{font-size:1.3rem;color:#4fc3f7;margin:0 0 6px 0}"
".sub{font-size:0.75rem;color:#888;margin-bottom:12px}"
".badge{display:inline-block;padding:4px 10px;border-radius:4px;font-size:0.75rem;margin-right:6px;background:#333;color:#aaa}"
".badge.ok{background:#1b5e20;color:#81c784}"
".badge.err{background:#b71c1c;color:#ef9a9a}"
".cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:10px;margin-bottom:14px}"
".card{background:#1e1e1e;border:1px solid #333;border-radius:6px;padding:10px}"
".label{font-size:0.65rem;color:#9e9e9e;text-transform:uppercase}"
".val{font-size:1.25rem;font-weight:700;margin-top:2px}"
".box{background:#1e1e1e;border:1px solid #333;border-radius:6px;padding:10px;margin-bottom:12px;position:relative}"
".box-title{font-size:0.8rem;color:#bbb;margin-bottom:6px}"
"canvas{width:100%;height:110px;display:block;cursor:crosshair}"
"#tip{position:fixed;display:none;background:#212121ee;border:1px solid #555;padding:8px 12px;border-radius:6px;font-size:0.75rem;pointer-events:none;z-index:999;box-shadow:0 4px 12px rgba(0,0,0,0.6)}"
"</style></head><body>"
"<h1>ESP32-S3-GEEK Telemetry Dashboard</h1>"
"<div class=\"sub\">Zephyr RTOS &bull; Dual-Core AMP (PRO_CPU + APP_CPU) &bull; FatFS CSV Logger &bull; SSE Live Stream</div>"
"<div style=\"margin-bottom:10px\">"
"<span id=\"badge-wifi\" class=\"badge ok\">Wi-Fi: Connected</span>"
"<span id=\"badge-sse\" class=\"badge\">SSE: 0 Clients</span>"
"<span id=\"badge-ntp\" class=\"badge\">NTP: Syncing...</span>"
"<span id=\"badge-sd\" class=\"badge ok\">SD: Logging</span>"
"<span id=\"badge-amp\" class=\"badge ok\">AMP: IPM Active</span>"
"</div>"
"<div style=\"display:flex;align-items:center;gap:10px;flex-wrap:wrap;background:#1e1e1e;padding:8px 12px;border-radius:6px;border:1px solid #333;margin-bottom:14px\">"
"<span style=\"font-size:0.75rem;font-weight:600;color:#aaa\">&#x1F4BE; CSV Log:</span>"
"<select id=\"csv-select\" style=\"background:#2a2a2a;color:#fff;border:1px solid #555;border-radius:4px;padding:3px 8px;font-size:0.75rem\"><option value=\"\">Loading logs...</option></select>"
"<button type=\"button\" onclick=\"downloadSelectedCsv()\" style=\"padding:3px 10px;background:#0288d1;color:#fff;border:none;border-radius:4px;font-size:0.75rem;font-weight:600;cursor:pointer\">&#x1F4E5; Download</button>"
"<span style=\"border-left:1px solid #444;height:18px;margin:0 4px\"></span>"
"<span style=\"font-size:0.75rem;font-weight:600;color:#aaa\">&#x1F5D1;&#xFE0F; Cleanup:</span>"
"<select id=\"clean-select\" style=\"background:#2a2a2a;color:#fff;border:1px solid #555;border-radius:4px;padding:3px 8px;font-size:0.75rem\"><option value=\"\">Select log...</option></select>"
"<button type=\"button\" onclick=\"deleteSelectedCsv()\" style=\"padding:3px 10px;background:#d32f2f;color:#fff;border:none;border-radius:4px;font-size:0.75rem;font-weight:600;cursor:pointer\">&#x2716; Delete</button>"
"</div>"
"<div class=\"cards\">"
"<div class=\"card\"><div class=\"label\">Motor Speed</div><div class=\"val\"><span id=\"v-rpm\">--</span> <small style=\"font-size:0.7rem;font-weight:400\">RPM</small></div></div>"
"<div class=\"card\"><div class=\"label\">Active PWM</div><div class=\"val\"><span id=\"v-pwm\">--</span> <small style=\"font-size:0.7rem;font-weight:400\">%</small></div></div>"
"<div class=\"card\"><div class=\"label\">Profile Stage</div><div class=\"val\" id=\"v-stage\" style=\"color:#aaa\">Idle</div></div>"
"<div class=\"card\"><div class=\"label\">Stage Progress</div><div class=\"val\"><span id=\"v-sec\">--</span> <small style=\"font-size:0.7rem;font-weight:400\" id=\"v-tot\">/ -- s</small></div></div>"
"<div class=\"card\"><div class=\"label\">Temperature</div><div class=\"val\"><span id=\"v-temp\">--</span> <small style=\"font-size:0.7rem;font-weight:400\">&deg;C</small></div></div>"
"<div class=\"card\"><div class=\"label\">Bus Voltage</div><div class=\"val\"><span id=\"v-volt\">--</span> <small style=\"font-size:0.7rem;font-weight:400\">V</small></div></div>"
"<div class=\"card\"><div class=\"label\">Current</div><div class=\"val\"><span id=\"v-curr\">--</span> <small style=\"font-size:0.7rem;font-weight:400\">mA</small></div></div>"
"</div>"
"<div class=\"box\">"
"<div class=\"box-title\">Motor RPM History (0 &ndash; 5500 RPM Scale)</div>"
"<canvas id=\"c-rpm\"></canvas>"
"</div>"
"<div class=\"box\">"
"<div class=\"box-title\">Voltage & Current History</div>"
"<canvas id=\"c-pwr\"></canvas>"
"</div>"
"<div class=\"box\">"
"<div class=\"box-title\">4-Stage PWM Profile Curve (Low &rarr; Ramp Up &rarr; High &rarr; Drop)</div>"
"<canvas id=\"c-curve\" style=\"height:130px;margin-bottom:10px\"></canvas>"
"<div style=\"display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:8px;margin-bottom:10px\">"
"<div><div class=\"label\">1. Low Time (s)</div><input id=\"t-low\" type=\"number\" min=\"0\" max=\"32768\" value=\"410\" style=\"width:100%;background:#2a2a2a;border:1px solid #444;color:#fff;padding:4px 6px;border-radius:4px;box-sizing:border-box\"></div>"
"<div><div class=\"label\">Target Low PWM (%)</div><input id=\"p-low\" type=\"number\" min=\"0\" max=\"100\" value=\"0\" style=\"width:100%;background:#2a2a2a;border:1px solid #444;color:#fff;padding:4px 6px;border-radius:4px;box-sizing:border-box\"></div>"
"<div><div class=\"label\">2. Ramp Up Time (s)</div><input id=\"t-up\" type=\"number\" min=\"0\" max=\"32768\" value=\"10\" style=\"width:100%;background:#2a2a2a;border:1px solid #444;color:#fff;padding:4px 6px;border-radius:4px;box-sizing:border-box\"></div>"
"<div><div class=\"label\">3. High Time (s)</div><input id=\"t-high\" type=\"number\" min=\"0\" max=\"32768\" value=\"170\" style=\"width:100%;background:#2a2a2a;border:1px solid #444;color:#fff;padding:4px 6px;border-radius:4px;box-sizing:border-box\"></div>"
"<div><div class=\"label\">Target High PWM (%)</div><input id=\"p-high\" type=\"number\" min=\"0\" max=\"100\" value=\"90\" style=\"width:100%;background:#2a2a2a;border:1px solid #444;color:#fff;padding:4px 6px;border-radius:4px;box-sizing:border-box\"></div>"
"<div><div class=\"label\">4. Drop Time (s)</div><input id=\"t-drop\" type=\"number\" min=\"0\" max=\"32768\" value=\"10\" style=\"width:100%;background:#2a2a2a;border:1px solid #444;color:#fff;padding:4px 6px;border-radius:4px;box-sizing:border-box\"></div>"
"</div>"
"<div style=\"display:flex;align-items:center;gap:10px;flex-wrap:wrap\">"
"<button id=\"btn-run\" onclick=\"startProfile()\" style=\"padding:6px 16px;background:#2e7d32;color:#fff;border:none;border-radius:4px;font-weight:600;cursor:pointer\">&#x25B6; Run Profile (Loop)</button>"
"<button id=\"btn-stop\" onclick=\"stopProfile()\" style=\"padding:6px 16px;background:#c62828;color:#fff;border:none;border-radius:4px;font-weight:600;cursor:pointer\">&#x23F9; Stop (0%)</button>"
"<span id=\"prof-status\" style=\"font-size:0.8rem;color:#81c784\">Status: Loop Active</span>"
"</div>"
"</div>"
"<div id=\"tip\"></div>"
"<script>"
"const MAX_PTS=300;let pts=[];"
"const cR=document.getElementById('c-rpm'),xR=cR.getContext('2d');"
"const cP=document.getElementById('c-pwr'),xP=cP.getContext('2d');"
"const cC=document.getElementById('c-curve'),xC=cC.getContext('2d');"
"const tip=document.getElementById('tip');"
"let hoverIdx=-1;"
"function resize(){cR.width=cR.clientWidth*window.devicePixelRatio||1;cR.height=cR.clientHeight*window.devicePixelRatio||1;cP.width=cP.clientWidth*window.devicePixelRatio||1;cP.height=cP.clientHeight*window.devicePixelRatio||1;cC.width=cC.clientWidth*window.devicePixelRatio||1;cC.height=cC.clientHeight*window.devicePixelRatio||1;render();drawCurve(-1);}"
"window.addEventListener('resize',resize);"
"function handleMove(e,c){"
"const r=c.getBoundingClientRect();"
"const x=(e.clientX-r.left)/r.width;"
"if(pts.length>1){"
"hoverIdx=Math.max(0,Math.min(pts.length-1,Math.round(x*(pts.length-1))));"
"tip.style.display='block';"
"tip.style.left=(e.clientX+14)+'px';"
"tip.style.top=(e.clientY+14)+'px';"
"const p=pts[hoverIdx];"
"tip.innerHTML='<b>'+(p.time||'Sample #'+p.id)+'</b><br>RPM: '+p.rpm+'<br>Temp: '+p.temp+' &deg;C<br>Bus: '+p.volt+' V<br>Current: '+p.curr+' mA';"
"render();"
"}}"
"function handleLeave(){hoverIdx=-1;tip.style.display='none';render();}"
"[cR,cP].forEach(c=>{"
"c.addEventListener('mousemove',e=>handleMove(e,c));"
"c.addEventListener('mouseleave',handleLeave);"
"c.addEventListener('touchmove',e=>{if(e.touches.length>0)handleMove(e.touches[0],c);},{passive:true});"
"c.addEventListener('touchend',handleLeave);"
"});"
"function drawGrid(ctx,w,h){ctx.strokeStyle='#262626';ctx.lineWidth=1;ctx.beginPath();for(let y=h*0.25;y<h;y+=h*0.25){ctx.moveTo(0,y);ctx.lineTo(w,y);}ctx.stroke();}"
"function render(){"
"const wR=cR.width,hR=cR.height,wP=cP.width,hP=cP.height;"
"xR.clearRect(0,0,wR,hR);xP.clearRect(0,0,wP,hP);"
"drawGrid(xR,wR,hR);drawGrid(xP,wP,hP);"
"if(pts.length<2)return;"
"let maxR=5500;pts.forEach(p=>{if(p.rpm>maxR)maxR=p.rpm;});"
"xR.fillStyle='#666';xR.font=(9*window.devicePixelRatio)+'px sans-serif';"
"xR.fillText(maxR+' RPM',6,12*window.devicePixelRatio);"
"xR.fillText('0 RPM',6,hR-4);"
"xR.strokeStyle='#4fc3f7';xR.lineWidth=2*window.devicePixelRatio;xR.beginPath();"
"pts.forEach((p,i)=>{const x=(i/(pts.length-1))*wR;const y=hR-(Math.min(p.rpm,maxR)/maxR)*hR;if(i===0)xR.moveTo(x,y);else xR.lineTo(x,y);});xR.stroke();"
"let maxV=15,minI=0,maxI=250;pts.forEach(p=>{if(p.volt>maxV)maxV=p.volt;if(p.curr>maxI)maxI=p.curr;if(p.curr<minI)minI=p.curr;});"
"const spanI=(maxI-minI)||1;"
"xP.fillStyle='#81c784';xP.font=(9*window.devicePixelRatio)+'px sans-serif';xP.fillText(maxV+' V',6,12*window.devicePixelRatio);"
"xP.fillStyle='#ffb74d';xP.fillText(maxI+' mA',wP-(55*window.devicePixelRatio),12*window.devicePixelRatio);"
"if(minI<0){xP.fillText(minI+' mA',wP-(55*window.devicePixelRatio),hP-4);const y0=hP-((0-minI)/spanI)*hP;xP.strokeStyle='#ffb74d44';xP.lineWidth=1;xP.setLineDash([2,2]);xP.beginPath();xP.moveTo(0,y0);xP.lineTo(wP,y0);xP.stroke();xP.setLineDash([]);}"
"xP.strokeStyle='#81c784';xP.lineWidth=2*window.devicePixelRatio;xP.beginPath();"
"pts.forEach((p,i)=>{const x=(i/(pts.length-1))*wP;const y=hP-(p.volt/maxV)*hP;if(i===0)xP.moveTo(x,y);else xP.lineTo(x,y);});xP.stroke();"
"xP.strokeStyle='#ffb74d';xP.lineWidth=2*window.devicePixelRatio;xP.beginPath();"
"pts.forEach((p,i)=>{const x=(i/(pts.length-1))*wP;const y=hP-((p.curr-minI)/spanI)*hP;if(i===0)xP.moveTo(x,y);else xP.lineTo(x,y);});xP.stroke();"
"if(hoverIdx>=0&&hoverIdx<pts.length){"
"const xR_c=(hoverIdx/(pts.length-1))*wR;"
"const xP_c=(hoverIdx/(pts.length-1))*wP;"
"[xR,xP].forEach((ctx,idx)=>{const x=(idx===0)?xR_c:xP_c;const h=(idx===0)?hR:hP;"
"ctx.strokeStyle='#ffffff66';ctx.lineWidth=1;ctx.setLineDash([4,4]);ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,h);ctx.stroke();ctx.setLineDash([]);});"
"}}"
"const secIds=['t-low','t-up','t-high','t-drop'];"
"const pwmIds=['p-low','p-high'];"
"secIds.forEach(id=>{"
"const el=document.getElementById(id);"
"el.addEventListener('input',()=>{"
"let v=parseInt(el.value);if(isNaN(v)||v<0)el.value=0;else if(v>32768)el.value=32768;"
"drawCurve(-1);saveConfig();"
"});});"
"pwmIds.forEach(id=>{"
"const el=document.getElementById(id);"
"el.addEventListener('input',()=>{"
"let v=parseInt(el.value);if(isNaN(v)||v<0)el.value=0;else if(v>100)el.value=100;"
"drawCurve(-1);saveConfig();"
"});});"
"let saveTimer=null;"
"function saveConfig(){"
"if(saveTimer)clearTimeout(saveTimer);"
"saveTimer=setTimeout(()=>{"
"const t1=parseInt(document.getElementById('t-low').value)||0;"
"const t2=parseInt(document.getElementById('t-up').value)||0;"
"const t3=parseInt(document.getElementById('t-high').value)||0;"
"const t4=parseInt(document.getElementById('t-drop').value)||0;"
"const pL=parseInt(document.getElementById('p-low').value)||0;"
"const pH=parseInt(document.getElementById('p-high').value)||0;"
"fetch('/api/profile/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'t_low='+t1+'&p_low='+pL+'&t_up='+t2+'&t_high='+t3+'&p_high='+pH+'&t_drop='+t4}).catch(e=>{});"
"},300);"
"}"
"function drawCurve(curSec){"
"const w=cC.width,h=cC.height;"
"xC.clearRect(0,0,w,h);drawGrid(xC,w,h);"
"const t1=parseInt(document.getElementById('t-low').value)||0;"
"const t2=parseInt(document.getElementById('t-up').value)||0;"
"const t3=parseInt(document.getElementById('t-high').value)||0;"
"const t4=parseInt(document.getElementById('t-drop').value)||0;"
"const pL=parseInt(document.getElementById('p-low').value)||0;"
"const pH=parseInt(document.getElementById('p-high').value)||0;"
"const total=t1+t2+t3+t4;"
"if(total<=0){xC.fillStyle='#666';xC.font='11px sans-serif';xC.fillText('Set durations > 0s to view curve',10,h/2);return;}"
"const toX=s=>(s/total)*(w-50)+35;"
"const toY=p=>h-16-(p/100)*(h-32);"
"const x0=toX(0),x1=toX(t1),x2=toX(t1+t2),x3=toX(t1+t2+t3),x4=toX(total);"
"const yL=toY(pL),yH=toY(pH);"
"xC.strokeStyle='#ffffff22';xC.lineWidth=1;xC.setLineDash([3,3]);"
"[x1,x2,x3].forEach(x=>{xC.beginPath();xC.moveTo(x,6);xC.lineTo(x,h-16);xC.stroke();});"
"xC.setLineDash([]);"
"xC.fillStyle='#888';xC.font=(10*window.devicePixelRatio)+'px sans-serif';"
"if(t1>0)xC.fillText('Low ('+pL+'%)',x0+4,16);"
"if(t2>0)xC.fillText('Ramp Up',x1+4,16);"
"if(t3>0)xC.fillText('High ('+pH+'%)',x2+4,16);"
"if(t4>0)xC.fillText('Drop',x3+4,16);"
"xC.fillStyle='#555';xC.fillText('100%',4,toY(100)+4);xC.fillText('0%',12,toY(0)+4);"
"const grad=xC.createLinearGradient(0,0,0,h);"
"grad.addColorStop(0,'#00e5ff55');grad.addColorStop(1,'#00e5ff05');"
"xC.fillStyle=grad;xC.beginPath();xC.moveTo(x0,toY(0));"
"xC.lineTo(x0,yL);xC.lineTo(x1,yL);xC.lineTo(x2,yH);xC.lineTo(x3,yH);xC.lineTo(x4,yL);xC.lineTo(x4,toY(0));"
"xC.closePath();xC.fill();"
"xC.strokeStyle='#00e5ff';xC.lineWidth=2.5*window.devicePixelRatio;"
"xC.beginPath();xC.moveTo(x0,yL);xC.lineTo(x1,yL);xC.lineTo(x2,yH);xC.lineTo(x3,yH);xC.lineTo(x4,yL);xC.stroke();"
"if(curSec>=0&&curSec<=total){"
"const cx=toX(curSec);"
"let cp=pL;"
"if(curSec<=t1)cp=pL;"
"else if(curSec<=t1+t2)cp=pL+(pH-pL)*(t2>0?(curSec-t1)/t2:1);"
"else if(curSec<=t1+t2+t3)cp=pH;"
"else cp=pH-(pH-pL)*(t4>0?(curSec-t1-t2-t3)/t4:1);"
"const cy=toY(cp);"
"xC.strokeStyle='#ff5252';xC.lineWidth=1.5;xC.setLineDash([2,2]);"
"xC.beginPath();xC.moveTo(cx,6);xC.lineTo(cx,h-16);xC.stroke();xC.setLineDash([]);"
"xC.fillStyle='#ff5252';xC.beginPath();xC.arc(cx,cy,5*window.devicePixelRatio,0,Math.PI*2);xC.fill();"
"xC.fillStyle='#fff';xC.fillText(Math.round(cp)+'% ('+curSec+'s)',cx+6,cy-4);"
"}}"
"function startProfile(){"
"const t1=parseInt(document.getElementById('t-low').value)||0;"
"const t2=parseInt(document.getElementById('t-up').value)||0;"
"const t3=parseInt(document.getElementById('t-high').value)||0;"
"const t4=parseInt(document.getElementById('t-drop').value)||0;"
"const pL=parseInt(document.getElementById('p-low').value)||0;"
"const pH=parseInt(document.getElementById('p-high').value)||0;"
"const total=t1+t2+t3+t4;"
"if(total<=0){alert('Total profile duration must be > 0s');return;}"
"document.getElementById('prof-status').innerText='Starting backend profile...';"
"fetch('/api/profile/start',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'t_low='+t1+'&p_low='+pL+'&t_up='+t2+'&t_high='+t3+'&p_high='+pH+'&t_drop='+t4})"
".then(r=>r.json()).then(j=>{"
"document.getElementById('prof-status').innerHTML='<b>Backend Profile (Loop) Running</b>';"
"}).catch(e=>{document.getElementById('prof-status').innerText='Communication error';});"
"}"
"function stopProfile(){"
"document.getElementById('prof-status').innerText='Stopping profile...';"
"fetch('/api/profile/stop',{method:'POST'})"
".then(r=>r.json()).then(j=>{"
"document.getElementById('prof-status').innerText='Status: Stopped (0% PWM)';"
"drawCurve(-1);"
"}).catch(e=>{document.getElementById('prof-status').innerText='Error stopping';});"
"}"
"resize();"
"fetch('/api/profile/config').then(r=>r.json()).then(cfg=>{"
"if(cfg.t_low!==undefined){"
"document.getElementById('t-low').value=cfg.t_low;"
"document.getElementById('p-low').value=cfg.p_low;"
"document.getElementById('t-up').value=cfg.t_up;"
"document.getElementById('t-high').value=cfg.t_high;"
"document.getElementById('p-high').value=cfg.p_high;"
"document.getElementById('t-drop').value=cfg.t_drop;"
"drawCurve(-1);"
"}"
"}).catch(e=>{});"
"fetch('/api/profile/status').then(r=>r.json()).then(st=>{"
"if(st.running)drawCurve(st.elapsed_sec);"
"}).catch(e=>{});"
"const evt=new EventSource('/events');"
"evt.onmessage=e=>{"
"try{"
"const d=JSON.parse(e.data);"
"pts.push(d);"
"if(pts.length>MAX_PTS)pts.shift();"
"document.getElementById('v-rpm').innerText=d.rpm;"
"document.getElementById('v-temp').innerText=d.temp;"
"document.getElementById('v-volt').innerText=d.volt;"
"document.getElementById('v-curr').innerText=d.curr;"
"if(d.pwm!==undefined)document.getElementById('v-pwm').innerText=d.pwm;"
"if(d.stage!==undefined){"
"const stNames=['Idle','Low Stage','Ramp Up','High Stage','Drop'];"
"const stColors=['#aaa','#4fc3f7','#ffb74d','#81c784','#e57373'];"
"const sName=stNames[d.stage]||('Stage '+d.stage);"
"const elS=document.getElementById('v-stage');"
"elS.innerText=sName;elS.style.color=stColors[d.stage]||'#fff';"
"if(d.stage>0){"
"document.getElementById('v-sec').innerText=(d.prof_sec||0)+'s';"
"document.getElementById('v-tot').innerText='/ '+(d.prof_tot||0)+' s';"
"document.getElementById('prof-status').innerHTML='<b style=\"color:'+(stColors[d.stage]||'#81c784')+'\">Loop Active &bull; '+sName+' ('+d.pwm+'% PWM) &bull; '+(d.prof_sec||0)+'s / '+(d.prof_tot||0)+'s</b>';"
"drawCurve(d.prof_sec);"
"}else{"
"document.getElementById('v-sec').innerText='--';"
"document.getElementById('v-tot').innerText='/ -- s';"
"document.getElementById('prof-status').innerText='Status: Idle ('+d.pwm+'% PWM)';"
"drawCurve(-1);"
"}"
"}"
"if(d.clients!==undefined)document.getElementById('badge-sse').innerText='SSE: '+d.clients+' Client'+(d.clients===1?'':'s');"
"if(d.ntp!==undefined){const nb=document.getElementById('badge-ntp');if(d.ntp){nb.className='badge ok';nb.innerText='NTP: Synced';}else{nb.className='badge';nb.innerText='NTP: Syncing...';}}"
"render();"
"}catch(err){}"
"};"
"evt.onerror=()=>{document.getElementById('badge-sse').className='badge err';document.getElementById('badge-sse').innerText='SSE: Reconnecting...';};"
"evt.onopen=()=>{document.getElementById('badge-sse').className='badge ok';};"
"function loadCsvList(){"
"fetch('/api/csv/list').then(r=>r.json()).then(files=>{"
"const dl=document.getElementById('csv-select'),cl=document.getElementById('clean-select');"
"if(!dl||!cl)return;"
"dl.innerHTML='';cl.innerHTML='<option value=\"\">Select log...</option>';"
"if(!files||files.length===0){dl.innerHTML='<option value=\"\">No logs found</option>';return;}"
"files.forEach(f=>{"
"const o1=document.createElement('option');o1.value=f.name;"
"o1.textContent=f.name+' ('+Math.round(f.size/1024)+' KB)'+(f.is_today?' [Today]':'');"
"if(f.is_today)o1.selected=true;dl.appendChild(o1);"
"const o2=document.createElement('option');o2.value=f.name;"
"o2.textContent=f.name+(f.is_today?' [Today: Recreate]':'');cl.appendChild(o2);"
"});"
"}).catch(e=>{});"
"}"
"function downloadSelectedCsv(){"
"const s=document.getElementById('csv-select');if(!s||!s.value)return;"
"const fn=s.value;"
"const a=document.createElement('a');"
"a.href='/download?file='+encodeURIComponent(fn);"
"a.download=fn;"
"a.style.display='none';"
"document.body.appendChild(a);"
"a.click();"
"setTimeout(()=>document.body.removeChild(a),1000);"
"}"
"function deleteSelectedCsv(){"
"const s=document.getElementById('clean-select');if(!s||!s.value){alert('Select a log file to delete');return;}"
"const fn=s.value;"
"if(!confirm('Delete '+fn+'?\\n(If today\\'s log, it will be cleared & recreated)'))return;"
"fetch('/api/csv/delete',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'file='+encodeURIComponent(fn)})"
".then(r=>r.json()).then(j=>{alert('File '+(j.recreated?'cleared & recreated':'deleted'));loadCsvList();}).catch(e=>{alert('Error deleting');});"
"}"
"loadCsvList();"
"</script></body></html>";

/* ========================================================================= */
/* FatFS SD Card Initialization with Auto-Mount Support                      */
/* ========================================================================= */
static int init_sdcard_fat32(void)
{
	if (sdcard_mounted) {
		return 0;
	}

	int res = fs_mount(&fs_mnt);
	if (res == 0) {
		printk("[SD Card] FAT32 Volume mounted successfully at %s\n", fs_mnt.mnt_point);
		sdcard_mounted = true;
		return 0;
	}

	printk("[SD Card] Mount attempt failed at %s (err: %d). No SD card or unreadable.\n",
	       fs_mnt.mnt_point, res);
	sdcard_mounted = false;
	return res;
}

static void prune_old_csv_files(void)
{
	k_mutex_lock(&sdcard_fs_mutex, K_FOREVER);
	struct fs_dir_t dirp;
	fs_dir_t_init(&dirp);
	int rc = fs_opendir(&dirp, "/SD:");
	if (rc != 0) {
		k_mutex_unlock(&sdcard_fs_mutex);
		return;
	}

	char names[32][32];
	int count = 0;
	struct fs_dirent entry;

	while (fs_readdir(&dirp, &entry) == 0 && entry.name[0] != '\0') {
		if (entry.type == FS_DIR_ENTRY_FILE &&
		    strncmp(entry.name, "telemetry_", 10) == 0 &&
		    strstr(entry.name, ".csv") != NULL) {
			if (count < 32) {
				strncpy(names[count], entry.name, sizeof(names[count]) - 1);
				names[count][sizeof(names[count]) - 1] = '\0';
				count++;
			}
		}
	}
	fs_closedir(&dirp);

	if (count <= MAX_CSV_FILES_RETAINED) {
		k_mutex_unlock(&sdcard_fs_mutex);
		return;
	}

	/* Sort ascending: oldest dates first (telemetry_YYYY-MM-DD.csv) */
	for (int i = 0; i < count - 1; i++) {
		for (int j = i + 1; j < count; j++) {
			if (strcmp(names[i], names[j]) > 0) {
				char temp[32];
				memcpy(temp, names[i], sizeof(temp));
				memcpy(names[i], names[j], sizeof(temp));
				memcpy(names[j], temp, sizeof(temp));
			}
		}
	}

	int files_to_delete = count - MAX_CSV_FILES_RETAINED;
	for (int i = 0; i < files_to_delete; i++) {
		char del_path[64];
		snprintf(del_path, sizeof(del_path), "/SD:/%s", names[i]);
		int del_rc = fs_unlink(del_path);
		printk("[SD Card] Rotation limit (%d files) reached: pruned oldest CSV '%s' (rc: %d)\n",
		       MAX_CSV_FILES_RETAINED, del_path, del_rc);
	}
	k_mutex_unlock(&sdcard_fs_mutex);
}

/* ------------------------------------------------------------------------- */
/* Thread 1: SD Card FAT32 CSV Logger                                        */
/* ------------------------------------------------------------------------- */
static void sdcard_csv_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[Thread 1: SD CSV Logger] Started (Priority %d)\n", PRIORITY_LOW);

	init_sdcard_fat32();

	char current_day_str[24] = "";
	fs_file_t_init(&g_csv_file);
	int64_t last_mount_attempt = k_uptime_get();

	while (1) {
		telemetry_full_t packet;
		k_msgq_get(&csv_msgq, &packet, K_FOREVER);

		if (!sdcard_mounted) {
			int64_t now = k_uptime_get();
			if (now - last_mount_attempt > 10000) {
				last_mount_attempt = now;
				init_sdcard_fat32();
			}
			if (!sdcard_mounted) {
				continue;
			}
		}

		const char *day_str = packet.date_str;
		if (strlen(day_str) == 0) {
			continue;
		}

		/* Daily file rotation: If day changed or first run */
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

				/* Write CSV header if creating a new file */
				if (!exists || entry.size == 0) {
					const char *hdr = "timestamp_iso,uptime_ms,sample_id,rpm,temp_c,bus_volt_v,curr_ma\n";
					fs_write(&g_csv_file, hdr, strlen(hdr));
					fs_sync(&g_csv_file);
				}
				k_mutex_unlock(&sdcard_fs_mutex);

				k_mutex_lock(&csv_path_mutex, K_FOREVER);
				strncpy(active_csv_filename, filepath, sizeof(active_csv_filename));
				k_mutex_unlock(&csv_path_mutex);

				printk("[SD Card] Opened/Rotated CSV log: %s\n", filepath);
			} else {
				k_mutex_unlock(&sdcard_fs_mutex);
				printk("[SD Card] Failed to open CSV file %s (err %d)\n", filepath, res);
			}
		}

		/* Append telemetry record */
		if (g_csv_file_is_open) {
			char line[128];
			int len = snprintf(line, sizeof(line), "%s,%lld,%u,%u,%d.%d,%u.%02u,%d\n",
			                   packet.iso_time,
			                   packet.uptime_ms,
			                   packet.sample_id,
			                   packet.motor_rpm,
			                   packet.temp_mc / 1000, abs(packet.temp_mc % 1000) / 100,
			                   packet.bus_mv / 1000, (packet.bus_mv % 1000) / 10,
			                   packet.current_ma);

			if (len > 0) {
				k_mutex_lock(&sdcard_fs_mutex, K_FOREVER);
				fs_write(&g_csv_file, line, len);
				if (packet.sample_id % CONFIG_APP_SAMPLE_RATE_HZ == 0) {
					fs_sync(&g_csv_file);
				}
				k_mutex_unlock(&sdcard_fs_mutex);
			}
		}
	}
}

static K_SEM_DEFINE(download_start_sem, 0, 1);
static volatile int g_download_client_fd = -1;
static volatile bool g_download_in_progress = false;
static char g_download_target_file[64] = "";
static uint8_t s_download_buf[4096];

static void handle_http_csv_download(int client_fd)
{
	char filepath[80] = "";
	k_mutex_lock(&csv_path_mutex, K_FOREVER);
	if (strlen(g_download_target_file) > 0) {
		snprintf(filepath, sizeof(filepath), "/SD:/%s", g_download_target_file);
		g_download_target_file[0] = '\0';
	} else if (strlen(active_csv_filename) > 0) {
		strncpy(filepath, active_csv_filename, sizeof(filepath) - 1);
		filepath[sizeof(filepath) - 1] = '\0';
	}
	k_mutex_unlock(&csv_path_mutex);

	if (strlen(filepath) == 0 || !sdcard_mounted) {
		const char *not_found =
			"HTTP/1.1 404 Not Found\r\n"
			"Content-Type: text/plain\r\n"
			"Connection: close\r\n\r\n"
			"No active CSV file available on SD card.\r\n";
		send_all(client_fd, not_found, strlen(not_found));
		close(client_fd);
		return;
	}

	/* Flush any pending writes so file size and contents are current */
	sdcard_flush_sync();

	k_mutex_lock(&sdcard_fs_mutex, K_FOREVER);
	struct fs_file_t rf;
	fs_file_t_init(&rf);
	int rc = fs_open(&rf, filepath, FS_O_READ);
	k_mutex_unlock(&sdcard_fs_mutex);

	if (rc != 0) {
		printk("[HTTP Download] fs_open failed for '%s' (err %d)\n", filepath, rc);
		const char *err_resp =
			"HTTP/1.1 500 Internal Server Error\r\n"
			"Content-Type: text/plain\r\n"
			"Connection: close\r\n\r\n"
			"Failed to open CSV file on SD card.\r\n";
		send_all(client_fd, err_resp, strlen(err_resp));
		close(client_fd);
		return;
	}

	int one = 1;
	zsock_setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

	const char *basename = strrchr(filepath, '/');
	if (basename) {
		basename++;
	} else {
		basename = "telemetry.csv";
	}

	char header[256];
	int hlen = snprintf(header, sizeof(header),
	                    "HTTP/1.1 200 OK\r\n"
	                    "Content-Type: text/csv\r\n"
	                    "Content-Disposition: attachment; filename=\"%s\"\r\n"
	                    "Connection: close\r\n\r\n",
	                    basename);
	if (send_all(client_fd, header, hlen) < 0) {
		k_mutex_lock(&sdcard_fs_mutex, K_FOREVER);
		fs_close(&rf);
		k_mutex_unlock(&sdcard_fs_mutex);
		close(client_fd);
		return;
	}

	uint32_t chunk_count = 0;
	while (1) {
		k_mutex_lock(&sdcard_fs_mutex, K_FOREVER);
		ssize_t bytes_read = fs_read(&rf, s_download_buf, sizeof(s_download_buf));
		k_mutex_unlock(&sdcard_fs_mutex);

		if (bytes_read <= 0) {
			break;
		}

		if (send_all(client_fd, s_download_buf, (size_t)bytes_read) < 0) {
			break;
		}

		chunk_count++;
		if ((chunk_count % 4) == 0) {
			/* Brief 1ms rest every 16 KB to let Wi-Fi buffers and other threads breathe */
			k_msleep(1);
		} else {
			k_yield();
		}
	}

	k_mutex_lock(&sdcard_fs_mutex, K_FOREVER);
	fs_close(&rf);
	k_mutex_unlock(&sdcard_fs_mutex);

	close(client_fd);
	printk("[HTTP] CSV file '%s' downloaded successfully\n", filepath);
}

static void csv_download_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[Thread 4: Background CSV Downloader] Started (Priority %d)\n", PRIORITY_DOWNLOAD);

	while (1) {
		k_sem_take(&download_start_sem, K_FOREVER);
		int fd = g_download_client_fd;
		if (fd >= 0) {
			handle_http_csv_download(fd);
			g_download_client_fd = -1;
		}
		g_download_in_progress = false;
	}
}

/* ========================================================================= */
/* Thread 2 (Wi-Fi, Auto-Reconnect & HTTP/SSE Web Server)                    */
/* ========================================================================= */
static bool apply_static_ip_if_configured(struct net_if *iface)
{
#if defined(CONFIG_APP_STATIC_IP)
	if (sizeof(CONFIG_APP_STATIC_IP) > 1 && CONFIG_APP_STATIC_IP[0] != '\0') {
		struct in_addr ip_addr;
		struct in_addr netmask;
		struct in_addr gw;

		if (net_addr_pton(AF_INET, CONFIG_APP_STATIC_IP, &ip_addr) < 0) {
			printk("[Wi-Fi] ERROR: Invalid static IP address format: '%s'\n", CONFIG_APP_STATIC_IP);
			return false;
		}

		/* Subnet mask (default 255.255.255.0 if not specified or invalid) */
		const char *nm_str = "255.255.255.0";
#if defined(CONFIG_APP_STATIC_NETMASK)
		if (sizeof(CONFIG_APP_STATIC_NETMASK) > 1 && CONFIG_APP_STATIC_NETMASK[0] != '\0') {
			nm_str = CONFIG_APP_STATIC_NETMASK;
		}
#endif
		if (net_addr_pton(AF_INET, nm_str, &netmask) < 0) {
			net_addr_pton(AF_INET, "255.255.255.0", &netmask);
		}

		/* Gateway */
#if defined(CONFIG_APP_STATIC_GW)
		if (sizeof(CONFIG_APP_STATIC_GW) > 1 && CONFIG_APP_STATIC_GW[0] != '\0') {
			if (net_addr_pton(AF_INET, CONFIG_APP_STATIC_GW, &gw) == 0) {
				net_if_ipv4_set_gw(iface, &gw);
			}
		}
#endif

		/* Add unicast static IP */
		struct net_if_addr *if_addr = net_if_ipv4_addr_add(iface, &ip_addr, NET_ADDR_MANUAL, 0);
		if (!if_addr) {
			if_addr = net_if_ipv4_addr_lookup(&ip_addr, &iface);
		}
		if (!if_addr) {
			printk("[Wi-Fi] ERROR: Failed to assign static IP '%s' to network interface!\n",
			       CONFIG_APP_STATIC_IP);
			return false;
		}

		/* Set netmask for this address */
		net_if_ipv4_set_netmask_by_addr(iface, &ip_addr, &netmask);

		/* Reconfigure DNS resolver if gateway or public DNS available */
#if defined(CONFIG_DNS_RESOLVER)
		const char *dns_servers[3] = { NULL, NULL, NULL };
		int dns_idx = 0;
#if defined(CONFIG_APP_STATIC_GW)
		if (sizeof(CONFIG_APP_STATIC_GW) > 1 && CONFIG_APP_STATIC_GW[0] != '\0') {
			dns_servers[dns_idx++] = CONFIG_APP_STATIC_GW;
		}
#endif
		dns_servers[dns_idx++] = "8.8.8.8";
		dns_servers[dns_idx] = NULL;
		dns_resolve_reconfigure(dns_resolve_get_default(), dns_servers, NULL, DNS_SOURCE_MANUAL);
#endif

		snprintk(assigned_ipv4_str, sizeof(assigned_ipv4_str), "%s", CONFIG_APP_STATIC_IP);
		wifi_has_ip = true;
		printk("[Wi-Fi] Static IPv4 configured: %s (Netmask: %s)\n", CONFIG_APP_STATIC_IP, nm_str);
		return true;
	}
#endif
	return false;
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
                                    uint64_t mgmt_event, struct net_if *iface)
{
	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT: {
		wifi_connecting = false;
		const struct wifi_status *status = (const struct wifi_status *)cb->info;
		if (!status) {
			printk("[Wi-Fi] Event: CONNECT_RESULT (no status data)\n");
			break;
		}
		if (status->status == 0) {
			wifi_connected = true;
			printk("\n============================================================\n");
			printk("[Wi-Fi] Connected successfully to '%s'!\n", CONFIG_APP_WIFI_SSID);
			printk("============================================================\n");

			if (apply_static_ip_if_configured(iface)) {
				printk("  [IP Address]   : %s\n", assigned_ipv4_str);
				printk("  [Web URL]      : http://%s/\n", assigned_ipv4_str);
				printk("  [SSE Stream]   : http://%s/events\n", assigned_ipv4_str);
				printk("============================================================\n\n");
				k_sem_give(&ntp_trigger_sem);
			} else {
				printk("[Wi-Fi] Link UP -> Requesting DHCP IPv4 lease from router...\n");
				net_dhcpv4_start(iface);
			}
		} else {
			wifi_connected = false;
			wifi_has_ip = false;
			assigned_ipv4_str[0] = '\0';
			int code = status ? status->status : -1;
			printk("\n============================================================\n");
			printk("[Wi-Fi] Connection failed with status: %d, will retry...\n", code);
			printk("============================================================\n");
		}
		break;
	}
	case NET_EVENT_WIFI_DISCONNECT_RESULT: {
		wifi_connected = false;
		wifi_connecting = false;
		wifi_has_ip = false;
		assigned_ipv4_str[0] = '\0';
		printk("\n============================================================\n");
		printk("[Wi-Fi] Disconnected from network! Auto-reconnect active...\n");
		printk("============================================================\n");
#if defined(CONFIG_APP_STATIC_IP)
		if (sizeof(CONFIG_APP_STATIC_IP) > 1 && CONFIG_APP_STATIC_IP[0] != '\0') {
			struct in_addr ip_addr;
			if (net_addr_pton(AF_INET, CONFIG_APP_STATIC_IP, &ip_addr) == 0) {
				net_if_ipv4_addr_rm(iface, &ip_addr);
			}
		}
#endif
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
		char ip_buf[NET_IPV4_ADDR_LEN] = "0.0.0.0";
		char nm_buf[NET_IPV4_ADDR_LEN] = "0.0.0.0";
		char gw_buf[NET_IPV4_ADDR_LEN] = "0.0.0.0";

		for (int i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
			if (cfg->ip.ipv4->unicast[i].ipv4.addr_type == NET_ADDR_DHCP ||
			    cfg->ip.ipv4->unicast[i].ipv4.addr_type == NET_ADDR_MANUAL ||
			    cfg->ip.ipv4->unicast[i].ipv4.is_used) {
				net_addr_ntop(AF_INET, &cfg->ip.ipv4->unicast[i].ipv4.address.in_addr,
				              ip_buf, sizeof(ip_buf));
				net_addr_ntop(AF_INET, &cfg->ip.ipv4->unicast[i].netmask,
				              nm_buf, sizeof(nm_buf));
				net_addr_ntop(AF_INET, &cfg->ip.ipv4->gw,
				              gw_buf, sizeof(gw_buf));

				snprintk(assigned_ipv4_str, sizeof(assigned_ipv4_str), "%s", ip_buf);
				wifi_has_ip = true;

				printk("\n============================================================\n");
				printk(">>> ESP32-S3-GEEK Network Ready & Web Dashboard LIVE! <<<\n");
				printk("============================================================\n");
				printk("  [IP Address]   : %s\n", ip_buf);
				printk("  [Subnet Mask]  : %s\n", nm_buf);
				printk("  [Gateway]      : %s\n", gw_buf);
				printk("  [Web URL]      : http://%s/\n", ip_buf);
				printk("  [SSE Stream]   : http://%s/events\n", ip_buf);
				printk("  [Download CSV] : http://%s/download\n", ip_buf);
				printk("  [Status API]   : http://%s/status\n", ip_buf);
				printk("============================================================\n\n");

				k_sem_give(&ntp_trigger_sem);
				break;
			}
		}
		break;
	}
	case NET_EVENT_IPV4_ADDR_DEL: {
		printk("[Wi-Fi] IPv4 address lease released / expired (%s)\n", assigned_ipv4_str);
		assigned_ipv4_str[0] = '\0';
		wifi_has_ip = false;
		break;
	}
	default:
		break;
	}
}

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

	wifi_attempt_count++;
	wifi_connecting = true;

	char mac_str[18] = "Unknown MAC";
	struct net_linkaddr *link_addr = net_if_get_link_addr(iface);
	if (link_addr && link_addr->len == 6) {
		snprintk(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
		         link_addr->addr[0], link_addr->addr[1], link_addr->addr[2],
		         link_addr->addr[3], link_addr->addr[4], link_addr->addr[5]);
	}

	enum wifi_security_type sec_type = WIFI_SECURITY_TYPE_NONE;
	const char *sec_str = "Open";

	if (strlen(CONFIG_APP_WIFI_PSK) > 0) {
		sec_type = WIFI_SECURITY_TYPE_PSK;
		sec_str = "WPA2-PSK";
	}

	printk("\n[Wi-Fi] -----------------------------------------------------\n");
	printk("[Wi-Fi] Initiating connection attempt #%u...\n", wifi_attempt_count);
	printk("[Wi-Fi]   Target SSID : %s\n", CONFIG_APP_WIFI_SSID);
	printk("[Wi-Fi]   Security    : %s\n", sec_str);
#if defined(CONFIG_APP_STATIC_IP)
	if (sizeof(CONFIG_APP_STATIC_IP) > 1 && CONFIG_APP_STATIC_IP[0] != '\0') {
		printk("[Wi-Fi]   IPv4 Mode   : Static IP (%s)\n", CONFIG_APP_STATIC_IP);
	} else {
		printk("[Wi-Fi]   IPv4 Mode   : DHCPv4 (Dynamic)\n");
	}
#else
	printk("[Wi-Fi]   IPv4 Mode   : DHCPv4 (Dynamic)\n");
#endif
	printk("[Wi-Fi]   Station MAC : %s\n", mac_str);
	printk("[Wi-Fi] -----------------------------------------------------\n");

	struct wifi_connect_req_params params = {
		.ssid = CONFIG_APP_WIFI_SSID,
		.ssid_length = strlen(CONFIG_APP_WIFI_SSID),
		.psk = CONFIG_APP_WIFI_PSK,
		.psk_length = strlen(CONFIG_APP_WIFI_PSK),
		.channel = 0,
		.security = sec_type,
	};

	printk("[Wi-Fi] Connecting to SSID: '%s'...\n", CONFIG_APP_WIFI_SSID);
	int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
	if (ret) {
		wifi_connecting = false;
		printk("[Wi-Fi] Connect request returned: %d\n", ret);
	}
}

static int get_query_param_int(const char *req, const char *param, int default_val)
{
	char needle[32];
	snprintf(needle, sizeof(needle), "%s=", param);
	const char *p = strstr(req, needle);
	if (!p) {
		return default_val;
	}
	p += strlen(needle);
	return atoi(p);
}

static bool get_query_param_str(const char *req, const char *param, char *out_buf, size_t out_size)
{
	char needle[32];
	snprintf(needle, sizeof(needle), "%s=", param);
	const char *p = strstr(req, needle);
	if (!p) {
		return false;
	}
	p += strlen(needle);
	size_t idx = 0;
	while (*p && *p != '&' && *p != ' ' && *p != '\r' && *p != '\n' && idx + 1 < out_size) {
		out_buf[idx++] = *p++;
	}
	out_buf[idx] = '\0';
	return (idx > 0);
}

static void wifi_http_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[Thread 2: Wi-Fi & Web Server] Started (Priority %d)\n", PRIORITY_WIFI);

	struct net_if *iface = net_if_get_default();

	net_mgmt_init_event_callback(&wifi_cb, wifi_mgmt_event_handler,
	                             NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_cb);

	net_mgmt_init_event_callback(&ipv4_cb, wifi_mgmt_event_handler,
	                             NET_EVENT_IPV4_ADDR_ADD | NET_EVENT_IPV4_ADDR_DEL |
	                             NET_EVENT_IPV4_DHCP_START | NET_EVENT_IPV4_DHCP_BOUND |
	                             NET_EVENT_IPV4_DHCP_STOP);
	net_mgmt_add_event_callback(&ipv4_cb);

	int server_fd = -1;
	sse_client_t sse_clients[MAX_SSE_CLIENTS];
	for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
		sse_clients[i].fd = -1;
		sse_clients[i].last_sent_id = 0;
	}

	/* Pre-configure static IP on interface so driver adopts it immediately */
	if (iface) {
		apply_static_ip_if_configured(iface);
	}

	int64_t last_reconnect_attempt = k_uptime_get();

	/* Trigger initial connection attempt */
	trigger_wifi_connect();

	while (1) {
		int64_t now = k_uptime_get();
		sync_time_to_app_cpu(now, false);

		/* Auto-reconnect supervisor: If Wi-Fi is lost, retry every 5 seconds (matching ESP32-C5) */
		if (!wifi_connected && (now - last_reconnect_attempt > 5000)) {
			last_reconnect_attempt = now;
			trigger_wifi_connect();
		}

		if (!wifi_has_ip) {
			if (server_fd >= 0) {
				close(server_fd);
				server_fd = -1;
			}
			for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
				if (sse_clients[i].fd >= 0) {
					close(sse_clients[i].fd);
					sse_clients[i].fd = -1;
				}
			}
			k_sleep(K_MSEC(500));
			continue;
		}

		if (server_fd < 0) {
			server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
			if (server_fd < 0) {
				printk("[HTTP Server] Socket create failed (errno %d)\n", errno);
				k_sleep(K_SECONDS(2));
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
				printk("[HTTP Server] Bind failed (errno %d)\n", errno);
				close(server_fd);
				server_fd = -1;
				k_sleep(K_SECONDS(2));
				continue;
			}

			if (listen(server_fd, 5) < 0) {
				printk("[HTTP Server] Listen failed (errno %d)\n", errno);
				close(server_fd);
				server_fd = -1;
				k_sleep(K_SECONDS(2));
				continue;
			}

			printk("[HTTP Server] Listening on port %d...\n", HTTP_PORT);
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

		int poll_ret = poll(pfd, num_fds, 200);

		if (poll_ret > 0 && (pfd[0].revents & (ZSOCK_POLLERR | ZSOCK_POLLHUP))) {
			printk("[HTTP Server] Server listening socket error, recycling...\n");
			close(server_fd);
			server_fd = -1;
			continue;
		}

		/* 1. Handle incoming HTTP connection */
		if (poll_ret > 0 && (pfd[0].revents & ZSOCK_POLLIN)) {
			struct sockaddr_in client_addr;
			socklen_t client_len = sizeof(client_addr);
			int client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len);

			if (client_fd >= 0) {
				char rx_buf[512];
				int rlen = recv(client_fd, rx_buf, sizeof(rx_buf) - 1, 0);
				if (rlen > 0) {
					rx_buf[rlen] = '\0';

					/* Match OPTIONS (CORS preflight) */
					if (strncmp(rx_buf, "OPTIONS", 7) == 0) {
						const char *cors_resp =
							"HTTP/1.1 204 No Content\r\n"
							"Access-Control-Allow-Origin: *\r\n"
							"Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
							"Access-Control-Allow-Headers: *\r\n"
							"Connection: close\r\n\r\n";
						send_all(client_fd, cors_resp, strlen(cors_resp));
						close(client_fd);
					}
					/* Match GET /events (Server-Sent Events) */
					else if (strncmp(rx_buf, "GET /events", 11) == 0) {
						int slot = -1;
						for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
							if (sse_clients[i].fd < 0) {
								slot = i;
								break;
							}
						}

						if (slot >= 0) {
							const char *sse_headers =
								"HTTP/1.1 200 OK\r\n"
								"Content-Type: text/event-stream\r\n"
								"Cache-Control: no-cache\r\n"
								"Connection: keep-alive\r\n"
								"Access-Control-Allow-Origin: *\r\n\r\n";

							if (send_all(client_fd, sse_headers, strlen(sse_headers)) == 0) {
								int one = 1;
								zsock_setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
								sse_clients[slot].fd = client_fd;
								sse_clients[slot].last_sent_id = 0;
								printk("[HTTP] SSE client #%d connected on fd %d (broadcast only)\n",
								       slot + 1, client_fd);
							} else {
								close(client_fd);
							}
						} else {
							const char *busy_resp =
								"HTTP/1.1 503 Service Unavailable\r\n"
								"Content-Type: text/plain\r\n"
								"Connection: close\r\n\r\n"
								"Max SSE clients reached\n";
							send_all(client_fd, busy_resp, strlen(busy_resp));
							close(client_fd);
						}
					}
					/* Match GET / or GET /index.html */
					else if (strncmp(rx_buf, "GET / ", 6) == 0 ||
					         strncmp(rx_buf, "GET /index.html", 15) == 0) {
						char header[128];
						int hlen = snprintf(header, sizeof(header),
						                    "HTTP/1.1 200 OK\r\n"
						                    "Content-Type: text/html; charset=utf-8\r\n"
						                    "Content-Length: %zu\r\n"
						                    "Connection: close\r\n\r\n",
						                    sizeof(HTTP_HTML_PAGE) - 1);
						send_all(client_fd, header, hlen);
						send_all(client_fd, HTTP_HTML_PAGE, sizeof(HTTP_HTML_PAGE) - 1);
						close(client_fd);
					}
					/* Match GET /download or GET /download.csv or GET /api/download/csv */
					else if (strncmp(rx_buf, "GET /download", 13) == 0 ||
					         strncmp(rx_buf, "GET /telemetry.csv", 18) == 0 ||
					         strncmp(rx_buf, "GET /api/download/csv", 21) == 0) {
						if (g_download_in_progress) {
							const char *busy_resp =
								"HTTP/1.1 503 Service Unavailable\r\n"
								"Content-Type: text/plain\r\n"
								"Connection: close\r\n\r\n"
								"Another CSV download is already in progress.\r\n";
							send_all(client_fd, busy_resp, strlen(busy_resp));
							close(client_fd);
						} else {
							char req_file[64] = "";
							if (get_query_param_str(rx_buf, "file", req_file, sizeof(req_file))) {
								const char *c = strrchr(req_file, '/');
								if (c) {
									c++;
								} else {
									c = req_file;
								}
								k_mutex_lock(&csv_path_mutex, K_FOREVER);
								strncpy(g_download_target_file, c, sizeof(g_download_target_file) - 1);
								g_download_target_file[sizeof(g_download_target_file) - 1] = '\0';
								k_mutex_unlock(&csv_path_mutex);
							} else {
								k_mutex_lock(&csv_path_mutex, K_FOREVER);
								g_download_target_file[0] = '\0';
								k_mutex_unlock(&csv_path_mutex);
							}
							g_download_client_fd = client_fd;
							g_download_in_progress = true;
							k_sem_give(&download_start_sem);
						}
					}
					/* Match GET /api/csv/list */
					else if (strncmp(rx_buf, "GET /api/csv/list", 17) == 0) {
						static char list_resp[2048];
						int offset = snprintf(list_resp, sizeof(list_resp), "[");

						if (sdcard_mounted) {
							k_mutex_lock(&sdcard_fs_mutex, K_FOREVER);
							struct fs_dir_t dirp;
							fs_dir_t_init(&dirp);
							int rc = fs_opendir(&dirp, "/SD:");
							if (rc == 0) {
								bool first = true;
								struct fs_dirent entry;
								while (fs_readdir(&dirp, &entry) == 0) {
									if (entry.name[0] == '\0') {
										break;
									}
									if (entry.type == FS_DIR_ENTRY_FILE) {
										size_t nlen = strlen(entry.name);
										if (nlen > 4 && strcmp(entry.name + nlen - 4, ".csv") == 0) {
											char today_base[64] = "";
											k_mutex_lock(&csv_path_mutex, K_FOREVER);
											const char *slash = strrchr(active_csv_filename, '/');
											if (slash) {
												strncpy(today_base, slash + 1, sizeof(today_base) - 1);
											} else {
												strncpy(today_base, active_csv_filename, sizeof(today_base) - 1);
											}
											k_mutex_unlock(&csv_path_mutex);

											bool is_today = (strlen(today_base) > 0 && strcmp(entry.name, today_base) == 0);

											int written = snprintf(list_resp + offset, sizeof(list_resp) - offset,
											                      "%s{\"name\":\"%s\",\"size\":%zu,\"is_today\":%s}",
											                      first ? "" : ",",
											                      entry.name,
											                      entry.size,
											                      is_today ? "true" : "false");
											if (written > 0 && offset + written < sizeof(list_resp) - 4) {
												offset += written;
												first = false;
											}
										}
									}
								}
								fs_closedir(&dirp);
							}
							k_mutex_unlock(&sdcard_fs_mutex);
						}

						if (offset < sizeof(list_resp) - 3) {
							list_resp[offset++] = ']';
							list_resp[offset++] = '\r';
							list_resp[offset++] = '\n';
							list_resp[offset] = '\0';
						}

						char header[160];
						int hlen = snprintf(header, sizeof(header),
						                    "HTTP/1.1 200 OK\r\n"
						                    "Content-Type: application/json\r\n"
						                    "Access-Control-Allow-Origin: *\r\n"
						                    "Content-Length: %d\r\n"
						                    "Connection: close\r\n\r\n", offset);
						send_all(client_fd, header, hlen);
						send_all(client_fd, list_resp, offset);
						close(client_fd);
					}
					/* Match POST /api/csv/delete or GET /api/csv/delete */
					else if (strncmp(rx_buf, "POST /api/csv/delete", 20) == 0 ||
					         strncmp(rx_buf, "GET /api/csv/delete", 19) == 0) {
						char target_file[64] = "";
						get_query_param_str(rx_buf, "file", target_file, sizeof(target_file));
						const char *clean_name = strrchr(target_file, '/');
						if (clean_name) {
							clean_name++;
						} else {
							clean_name = target_file;
						}

						bool success = false;
						bool recreated = false;

						if (strlen(clean_name) > 4 && strcmp(clean_name + strlen(clean_name) - 4, ".csv") == 0 && sdcard_mounted) {
							char fullpath[80];
							snprintf(fullpath, sizeof(fullpath), "/SD:/%s", clean_name);

							char today_path[64] = "";
							k_mutex_lock(&csv_path_mutex, K_FOREVER);
							strncpy(today_path, active_csv_filename, sizeof(today_path) - 1);
							k_mutex_unlock(&csv_path_mutex);

							bool is_today = (strlen(today_path) > 0 && strcmp(fullpath, today_path) == 0);

							k_mutex_lock(&sdcard_fs_mutex, K_FOREVER);
							if (is_today) {
								if (g_csv_file_is_open) {
									fs_close(&g_csv_file);
									g_csv_file_is_open = false;
								}
								fs_unlink(fullpath);
								int res = fs_open(&g_csv_file, fullpath, FS_O_CREATE | FS_O_WRITE);
								if (res == 0) {
									const char *hdr = "timestamp_iso,uptime_ms,sample_id,rpm,temp_c,bus_volt_v,curr_ma\n";
									fs_write(&g_csv_file, hdr, strlen(hdr));
									fs_sync(&g_csv_file);
									g_csv_file_is_open = true;
									success = true;
									recreated = true;
									printk("[HTTP] Recreated active today CSV: %s\n", fullpath);
								}
							} else {
								int ures = fs_unlink(fullpath);
								if (ures == 0) {
									success = true;
									printk("[HTTP] Deleted old CSV: %s\n", fullpath);
								}
							}
							k_mutex_unlock(&sdcard_fs_mutex);
						}

						char body[128];
						int blen = snprintf(body, sizeof(body),
						                    "{\"status\":\"%s\",\"recreated\":%s}\r\n",
						                    success ? "ok" : "error",
						                    recreated ? "true" : "false");
						char header[160];
						int hlen = snprintf(header, sizeof(header),
						                    "HTTP/1.1 %s\r\n"
						                    "Content-Type: application/json\r\n"
						                    "Access-Control-Allow-Origin: *\r\n"
						                    "Content-Length: %d\r\n"
						                    "Connection: close\r\n\r\n",
						                    success ? "200 OK" : "400 Bad Request", blen);
						send_all(client_fd, header, hlen);
						send_all(client_fd, body, blen);
						close(client_fd);
					}
					/* Match POST /api/profile/config or GET /api/profile/config */
					else if (strncmp(rx_buf, "POST /api/profile/config", 24) == 0 ||
					         strncmp(rx_buf, "GET /api/profile/config", 23) == 0) {
						if (strstr(rx_buf, "t_low=")) {
							int t_low  = get_query_param_int(rx_buf, "t_low", g_active_profile_cfg.t_low_sec);
							int p_low  = get_query_param_int(rx_buf, "p_low", g_active_profile_cfg.p_low_pct);
							int t_up   = get_query_param_int(rx_buf, "t_up", g_active_profile_cfg.t_up_sec);
							int t_high = get_query_param_int(rx_buf, "t_high", g_active_profile_cfg.t_high_sec);
							int p_high = get_query_param_int(rx_buf, "p_high", g_active_profile_cfg.p_high_pct);
							int t_drop = get_query_param_int(rx_buf, "t_drop", g_active_profile_cfg.t_drop_sec);

							if (t_low < 0) { t_low = 0; } else if (t_low > 32768) { t_low = 32768; }
							if (t_up < 0) { t_up = 0; } else if (t_up > 32768) { t_up = 32768; }
							if (t_high < 0) { t_high = 0; } else if (t_high > 32768) { t_high = 32768; }
							if (t_drop < 0) { t_drop = 0; } else if (t_drop > 32768) { t_drop = 32768; }
							if (p_low < 0) { p_low = 0; } else if (p_low > 100) { p_low = 100; }
							if (p_high < 0) { p_high = 0; } else if (p_high > 100) { p_high = 100; }

							g_active_profile_cfg.t_low_sec = (uint16_t)t_low;
							g_active_profile_cfg.p_low_pct = (uint8_t)p_low;
							g_active_profile_cfg.t_up_sec = (uint16_t)t_up;
							g_active_profile_cfg.t_high_sec = (uint16_t)t_high;
							g_active_profile_cfg.p_high_pct = (uint8_t)p_high;
							g_active_profile_cfg.t_drop_sec = (uint16_t)t_drop;
							g_active_profile_cfg.action = PWM_ACTION_SET_CONFIG_ONLY;

							if (ipm_dev && device_is_ready(ipm_dev)) {
								ipm_send(ipm_dev, 0, IPM_CHANNEL_PWM_CTRL, &g_active_profile_cfg, sizeof(g_active_profile_cfg));
							}
						}

						char body[128];
						int blen = snprintf(body, sizeof(body),
						                    "{\"t_low\":%u,\"p_low\":%u,\"t_up\":%u,\"t_high\":%u,\"p_high\":%u,\"t_drop\":%u}\r\n",
						                    g_active_profile_cfg.t_low_sec, g_active_profile_cfg.p_low_pct,
						                    g_active_profile_cfg.t_up_sec, g_active_profile_cfg.t_high_sec,
						                    g_active_profile_cfg.p_high_pct, g_active_profile_cfg.t_drop_sec);
						char header[160];
						int hlen = snprintf(header, sizeof(header),
						                    "HTTP/1.1 200 OK\r\n"
						                    "Content-Type: application/json\r\n"
						                    "Access-Control-Allow-Origin: *\r\n"
						                    "Content-Length: %d\r\n"
						                    "Connection: close\r\n\r\n", blen);
						send_all(client_fd, header, hlen);
						send_all(client_fd, body, blen);
						close(client_fd);
					}
					/* Match GET /api/profile/status */
					else if (strncmp(rx_buf, "GET /api/profile/status", 23) == 0) {
						telemetry_full_t cur;
						k_spinlock_key_t key = k_spin_lock(&telemetry_spinlock);
						cur = latest_telemetry;
						k_spin_unlock(&telemetry_spinlock, key);

						const char *st_names[] = {"Idle", "Low", "Ramp Up", "High", "Drop"};
						const char *st_name = (cur.profile_stage <= 4) ? st_names[cur.profile_stage] : "Unknown";

						char body[160];
						int blen = snprintf(body, sizeof(body),
						                    "{\"running\":%s,\"stage\":%u,\"stage_name\":\"%s\",\"elapsed_sec\":%u,\"total_sec\":%u,\"cur_pwm\":%u,\"rpm\":%u}\r\n",
						                    (cur.profile_stage > 0) ? "true" : "false",
						                    cur.profile_stage, st_name,
						                    cur.profile_elapsed_sec, cur.profile_total_sec,
						                    cur.pwm_pct, cur.motor_rpm);
						char header[160];
						int hlen = snprintf(header, sizeof(header),
						                    "HTTP/1.1 200 OK\r\n"
						                    "Content-Type: application/json\r\n"
						                    "Access-Control-Allow-Origin: *\r\n"
						                    "Content-Length: %d\r\n"
						                    "Connection: close\r\n\r\n", blen);
						send_all(client_fd, header, hlen);
						send_all(client_fd, body, blen);
						close(client_fd);
					}
					/* Match POST /api/profile/start or GET /api/profile/start */
					else if (strncmp(rx_buf, "POST /api/profile/start", 23) == 0 ||
					         strncmp(rx_buf, "GET /api/profile/start", 22) == 0 ||
					         ((strncmp(rx_buf, "POST /api/profile", 17) == 0 || strncmp(rx_buf, "GET /api/profile", 16) == 0) &&
					          strstr(rx_buf, "action=start"))) {
						if (strstr(rx_buf, "t_low=")) {
							int t_low  = get_query_param_int(rx_buf, "t_low", g_active_profile_cfg.t_low_sec);
							int p_low  = get_query_param_int(rx_buf, "p_low", g_active_profile_cfg.p_low_pct);
							int t_up   = get_query_param_int(rx_buf, "t_up", g_active_profile_cfg.t_up_sec);
							int t_high = get_query_param_int(rx_buf, "t_high", g_active_profile_cfg.t_high_sec);
							int p_high = get_query_param_int(rx_buf, "p_high", g_active_profile_cfg.p_high_pct);
							int t_drop = get_query_param_int(rx_buf, "t_drop", g_active_profile_cfg.t_drop_sec);

							if (t_low < 0) { t_low = 0; } else if (t_low > 32768) { t_low = 32768; }
							if (t_up < 0) { t_up = 0; } else if (t_up > 32768) { t_up = 32768; }
							if (t_high < 0) { t_high = 0; } else if (t_high > 32768) { t_high = 32768; }
							if (t_drop < 0) { t_drop = 0; } else if (t_drop > 32768) { t_drop = 32768; }
							if (p_low < 0) { p_low = 0; } else if (p_low > 100) { p_low = 100; }
							if (p_high < 0) { p_high = 0; } else if (p_high > 100) { p_high = 100; }

							g_active_profile_cfg.t_low_sec = (uint16_t)t_low;
							g_active_profile_cfg.p_low_pct = (uint8_t)p_low;
							g_active_profile_cfg.t_up_sec = (uint16_t)t_up;
							g_active_profile_cfg.t_high_sec = (uint16_t)t_high;
							g_active_profile_cfg.p_high_pct = (uint8_t)p_high;
							g_active_profile_cfg.t_drop_sec = (uint16_t)t_drop;
						}
						g_active_profile_cfg.action = PWM_ACTION_START_PROFILE;
						if (ipm_dev && device_is_ready(ipm_dev)) {
							ipm_send(ipm_dev, 0, IPM_CHANNEL_PWM_CTRL, &g_active_profile_cfg, sizeof(g_active_profile_cfg));
						}
						const char *resp =
							"HTTP/1.1 200 OK\r\n"
							"Content-Type: application/json\r\n"
							"Access-Control-Allow-Origin: *\r\n"
							"Content-Length: 20\r\n"
							"Connection: close\r\n\r\n"
							"{\"status\":\"started\"}\r\n";
						send_all(client_fd, resp, strlen(resp));
						close(client_fd);
					}
					/* Match POST /api/profile/stop or GET /api/profile/stop */
					else if (strncmp(rx_buf, "POST /api/profile/stop", 22) == 0 ||
					         strncmp(rx_buf, "GET /api/profile/stop", 21) == 0 ||
					         ((strncmp(rx_buf, "POST /api/profile", 17) == 0 || strncmp(rx_buf, "GET /api/profile", 16) == 0) &&
					          strstr(rx_buf, "action=stop"))) {
						g_active_profile_cfg.action = PWM_ACTION_STOP_PROFILE;
						if (ipm_dev && device_is_ready(ipm_dev)) {
							ipm_send(ipm_dev, 0, IPM_CHANNEL_PWM_CTRL, &g_active_profile_cfg, sizeof(g_active_profile_cfg));
						}
						const char *resp =
							"HTTP/1.1 200 OK\r\n"
							"Content-Type: application/json\r\n"
							"Access-Control-Allow-Origin: *\r\n"
							"Content-Length: 20\r\n"
							"Connection: close\r\n\r\n"
							"{\"status\":\"stopped\"}\r\n";
						send_all(client_fd, resp, strlen(resp));
						close(client_fd);
					}
					/* Match POST /api/pwm or GET /api/pwm */
					else if (strncmp(rx_buf, "POST /api/pwm", 13) == 0 ||
					         strncmp(rx_buf, "GET /api/pwm", 12) == 0) {
						int target_val = get_query_param_int(rx_buf, "val", 50);
						if (target_val < 0) target_val = 0;
						if (target_val > 100) target_val = 100;

						ipm_pwm_ctrl_t cmd = {
							.action = PWM_ACTION_MANUAL_PWM,
							.manual_pwm = (uint8_t)target_val,
						};
						if (ipm_dev && device_is_ready(ipm_dev)) {
							ipm_send(ipm_dev, 0, IPM_CHANNEL_PWM_CTRL, &cmd, sizeof(cmd));
						}
						char body[64];
						int blen = snprintf(body, sizeof(body), "{\"status\":\"ok\",\"pwm\":%d}\r\n", target_val);
						char header[160];
						int hlen = snprintf(header, sizeof(header),
						                    "HTTP/1.1 200 OK\r\n"
						                    "Content-Type: application/json\r\n"
						                    "Access-Control-Allow-Origin: *\r\n"
						                    "Content-Length: %d\r\n"
						                    "Connection: close\r\n\r\n", blen);
						send_all(client_fd, header, hlen);
						send_all(client_fd, body, blen);
						close(client_fd);
					}
					/* 404 for unknown endpoints */
					else {
						const char *not_found =
							"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
						send_all(client_fd, not_found, strlen(not_found));
						close(client_fd);
					}
				} else {
					close(client_fd);
				}
			}
		}

		/* 2. Check SSE client disconnects */
		if (poll_ret > 0) {
			for (int k = 1; k < num_fds; k++) {
				int c_idx = pfd_to_client[k - 1];
				int cfd = sse_clients[c_idx].fd;
				if (cfd < 0) {
					continue;
				}

				if (pfd[k].revents & (ZSOCK_POLLERR | ZSOCK_POLLHUP)) {
					printk("[HTTP] SSE client #%d (fd %d) disconnected\n", c_idx + 1, cfd);
					close(cfd);
					sse_clients[c_idx].fd = -1;
				} else if (pfd[k].revents & ZSOCK_POLLIN) {
					char probe[16];
					int r = recv(cfd, probe, sizeof(probe), ZSOCK_MSG_DONTWAIT);
					if (r <= 0) {
						if (r == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
							printk("[HTTP] SSE client #%d (fd %d) disconnected (EOF)\n",
							       c_idx + 1, cfd);
							close(cfd);
							sse_clients[c_idx].fd = -1;
						}
					}
				}
			}
		}

		/* 3. Push latest telemetry frame to active SSE clients */
		telemetry_full_t cur;
		bool has_new_packet = false;

		k_spinlock_key_t key = k_spin_lock(&telemetry_spinlock);
		if (has_telemetry) {
			cur = latest_telemetry;
			has_new_packet = true;
		}
		k_spin_unlock(&telemetry_spinlock, key);

		if (has_new_packet) {
			int active_count = 0;
			int pending = 0;

			for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
				if (sse_clients[i].fd >= 0) {
					active_count++;
					if (sse_clients[i].last_sent_id != cur.sample_id) {
						pending++;
					}
				}
			}

			if (pending > 0) {
				int32_t temp_int  = cur.temp_mc / 1000;
				int32_t temp_frac = abs(cur.temp_mc % 1000) / 100;
				uint32_t volt_int  = cur.bus_mv / 1000;
				uint32_t volt_frac = (cur.bus_mv % 1000) / 10;

				char sse_data[320];
				int sse_len = snprintf(sse_data, sizeof(sse_data),
				                       "data: {\"id\":%u,\"rpm\":%u,\"pwm\":%u,\"stage\":%u,\"prof_sec\":%u,\"prof_tot\":%u,\"temp\":%d.%d,\"volt\":%u.%02u,\"curr\":%d,\"uptime\":%lld,\"time\":\"%s\",\"clients\":%d,\"ntp\":%s}\n\n",
				                       cur.sample_id,
				                       cur.motor_rpm,
				                       cur.pwm_pct,
				                       cur.profile_stage,
				                       cur.profile_elapsed_sec,
				                       cur.profile_total_sec,
				                       temp_int, temp_frac,
				                       volt_int, volt_frac,
				                       cur.current_ma,
				                       cur.uptime_ms,
				                       cur.iso_time,
				                       active_count,
				                       ntp_synced ? "true" : "false");

				for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
					if (sse_clients[i].fd >= 0 &&
					    sse_clients[i].last_sent_id != cur.sample_id) {
						int r = send_sse_frame(sse_clients[i].fd, sse_data, sse_len);
						if (r < 0) {
							printk("[HTTP] SSE client #%d disconnected (err %d), closing\n", i + 1, errno);
							close(sse_clients[i].fd);
							sse_clients[i].fd = -1;
						} else if (r > 0) {
							sse_clients[i].last_sent_id = cur.sample_id;
						}
					}
				}
			}
		}
	}
}

/* ========================================================================= */
/* Thread 3: Decoupled Background NTP Synchronization                        */
/* ========================================================================= */
static void ntp_sync_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[Thread 3: NTP Client] Started (Priority %d)\n", PRIORITY_NTP);

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
			printk("[NTP] Time synchronized! Current Local (UTC+8): %s (epoch %lld)\n",
			       iso, (long long)ntp_epoch_sec);
			sync_time_to_app_cpu(k_uptime_get(), true);
		} else {
			printk("[NTP] Sync attempt failed (%d), will retry in 30s\n", ret);
			k_sleep(K_SECONDS(30));
			if (wifi_has_ip) {
				k_sem_give(&ntp_trigger_sem);
			}
			continue;
		}

		/* Re-sync NTP every 1 hour to correct hardware RTC drift */
		k_sleep(K_SECONDS(3600));
		if (wifi_has_ip) {
			k_sem_give(&ntp_trigger_sem);
		}
	}
}

/* ========================================================================= */
/* Thread 0: Core 1 (APP_CPU) IPC Print Console Consumer                     */
/* ========================================================================= */
static void ipc_console_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	ipc_print_packet_t pkt;

	while (1) {
		if (k_msgq_get(&ipc_print_msgq, &pkt, K_FOREVER) == 0) {
			printk("%s", pkt.msg);
		}
	}
}

/* Static Thread Allocations */
K_THREAD_DEFINE(thread_ipc_console, IPC_CONSOLE_STACK_SIZE, ipc_console_thread_fn,
                NULL, NULL, NULL, PRIORITY_IPC_CONSOLE, 0, 0);

K_THREAD_DEFINE(thread_csv, CSV_STACK_SIZE, sdcard_csv_thread_fn,
                NULL, NULL, NULL, PRIORITY_LOW, 0, 0);

K_THREAD_DEFINE(thread_wifi_http, WIFI_STACK_SIZE, wifi_http_thread_fn,
                NULL, NULL, NULL, PRIORITY_WIFI, 0, 0);

K_THREAD_DEFINE(thread_csv_download, DOWNLOAD_STACK_SIZE, csv_download_thread_fn,
                NULL, NULL, NULL, PRIORITY_DOWNLOAD, 0, 0);

K_THREAD_DEFINE(thread_ntp, NTP_STACK_SIZE, ntp_sync_thread_fn,
                NULL, NULL, NULL, PRIORITY_NTP, 0, 0);

/* ========================================================================= */
/* Main Application Entry Point                                              */
/* ========================================================================= */
int main(void)
{
	printk("=================================================================\n");
	printk("Waveshare ESP32-S3-GEEK: PRO_CPU (Core 0) Primary Multi-Core Host\n");
	printk("Architecture: Dual-Core Xtensa LX7 AMP via Sysbuild\n");
	printk("Subsystems: Wi-Fi HTTP/SSE, FAT32 SDCard Logger, NTP Clock Sync\n");
	printk("=================================================================\n");

	ipm_dev = DEVICE_DT_GET(DT_NODELABEL(ipm0));
	if (!ipm_dev || !device_is_ready(ipm_dev)) {
		printk("[PRO_CPU] WARNING: IPM device not ready at boot\n");
	} else {
		ipm_register_callback(ipm_dev, ipm_receive_callback, NULL);
		ipm_set_enabled(ipm_dev, 1);
		printk("[PRO_CPU] IPM receiver registered on channels %d (telemetry) and %d (console)\n",
		       IPM_CHANNEL_APP_TELEMETRY, IPM_CHANNEL_APP_PRINT);
		sync_time_to_app_cpu(k_uptime_get(), true);
	}

	while (1) {
		k_sleep(K_SECONDS(30));
		const char *wifi_state_str;
		if (wifi_has_ip) {
			wifi_state_str = assigned_ipv4_str;
		} else if (wifi_connected) {
			wifi_state_str = "Associated (Waiting for DHCP)";
		} else {
			wifi_state_str = "Disconnected (Retrying)";
		}

		printk("[PRO_CPU Supervisor] Uptime: %lld s | Wi-Fi: %s | NTP: %s | Attempts: %u\n",
		       k_uptime_get() / 1000,
		       wifi_state_str,
		       ntp_synced ? "Synced" : "Pending",
		       wifi_attempt_count);
	}

	return 0;
}
