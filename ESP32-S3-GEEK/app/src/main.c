/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Waveshare ESP32-S3-GEEK: Multi-Threaded Sensor Pipeline with FAT32 SDCard CSV Logger,
 * NTP Clock Synchronization, and Multi-Client SSE Web Dashboard.
 *
 * Architecture:
 *   [HW Timer ISR] -> signals sem -> [Thread 1: Speed Sampler (Priority 3)]
 *                                          │ (motor_speed_msgq)
 *                                          ▼
 *                                     [Thread 2: Sensor Aggregator (Priority 5)]
 *                                     ├── (csv_msgq)       -> [Thread 3: SDCard CSV Logger (Priority 7)]
 *                                     └── (condvar/mutex)  -> [Thread 4: Wi-Fi/NTP/SSE Server (Priority 8)]
 *
 * Features:
 *   1. FAT32 SD Card Support with auto-format on startup if unformatted.
 *   2. Daily auto-rotating CSV file logging: /SD:/telemetry_YYYY-MM-DD.csv
 *   3. NTP client synchronization for absolute wall-clock timestamps in CSV & Web UI.
 *   4. Multi-client SSE server (/events) streaming up to 1-hour historical data from CSV on connect,
 *      followed by real-time sensor updates.
 *   5. Zero-dependency Canvas Dashboard with interactive hover tooltip & crosshair inspect.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
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
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/sntp.h>

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

#ifndef CONFIG_APP_SAMPLE_RATE_HZ
#define CONFIG_APP_SAMPLE_RATE_HZ   10
#endif

#ifndef CONFIG_APP_MOTOR_PULSES_PER_REV
#define CONFIG_APP_MOTOR_PULSES_PER_REV 2
#endif

#define TIMER_INTERVAL_US           (1000000 / CONFIG_APP_SAMPLE_RATE_HZ)
#define MAX_HIST_REPLAY_SAMPLES     (3600 * CONFIG_APP_SAMPLE_RATE_HZ)  /* 1 hour history */

#define PRIORITY_HIGH               3     /* Thread 1: Motor Speed Sampler (FG Counter) */
#define PRIORITY_MEDIUM             5     /* Thread 2: Sensor Aggregator (I2C/INA219)   */
#define PRIORITY_LOW                7     /* Thread 3: SD Card CSV Logger               */
#define PRIORITY_WIFI               8     /* Thread 4: Wi-Fi & Web Server               */
#define PRIORITY_NTP                10    /* Thread 5: Background NTP Client            */

#define THREAD_STACK_SIZE           2048
#define CSV_STACK_SIZE              4096
#define WIFI_STACK_SIZE             6144
#define NTP_STACK_SIZE              3072

#define TIMER_NODE                  DT_ALIAS(app_timer)

BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(TIMER_NODE),
             "Devicetree error: 'app-timer' alias is missing or disabled!");

/* Hardware Devicetree Specs for Sensor Acquisition */
static const struct gpio_dt_spec fg_spec = GPIO_DT_SPEC_GET_OR(DT_ALIAS(fg_pin), gpios, {0});
static const struct pwm_dt_spec pwm_motor = PWM_DT_SPEC_GET_OR(DT_ALIAS(motor_pwm), {0});
static const struct device *const ina219_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(ina219));

static volatile uint32_t g_fg_pulse_counter = 0;
static struct gpio_callback fg_gpio_cb;

static void fg_gpio_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	g_fg_pulse_counter++;
}

/* ========================================================================= */
/* Data Structures & Shared State                                            */
/* ========================================================================= */
typedef struct {
	uint32_t pulse_count;
	uint32_t rpm;
	int64_t  timestamp_ms;
} motor_speed_sample_t;

typedef struct {
	uint32_t sample_id;
	uint32_t motor_rpm;
	int32_t  temp_mc;      /* milli-Celsius (e.g. 42100 = 42.1 C) */
	uint32_t bus_mv;       /* milli-Volts (e.g. 12050 = 12.05 V)  */
	uint32_t current_ma;   /* milli-Amps (e.g. 840 = 840 mA)      */
	int64_t  uptime_ms;
	char     iso_time[32]; /* "YYYY-MM-DDTHH:MM:SSZ"              */
} telemetry_packet_t;

typedef struct {
	int fd;
	uint32_t last_sent_id;
} sse_client_t;

/* Message Queues */
K_MSGQ_DEFINE(motor_speed_msgq, sizeof(motor_speed_sample_t), 4, 4);
K_MSGQ_DEFINE(csv_msgq, sizeof(telemetry_packet_t), 8, 4);

/* Semaphores & Synchronization */
static K_SEM_DEFINE(timer_tick_sem, 0, 1);
static K_MUTEX_DEFINE(telemetry_mutex);
static K_CONDVAR_DEFINE(telemetry_cv);
static telemetry_packet_t latest_telemetry;
static bool has_telemetry = false;

/* Active CSV Path shared between logger & SSE replay */
static K_MUTEX_DEFINE(csv_path_mutex);
static char active_csv_filename[64] = "";

/* Real-Time Clock / NTP State */
static K_MUTEX_DEFINE(time_mutex);
static int64_t ntp_epoch_sec = 0;
static int64_t ntp_sync_uptime = 0;
static bool ntp_synced = false;
static K_SEM_DEFINE(ntp_trigger_sem, 0, 1);

/* Wi-Fi & Network State */
static volatile bool wifi_connected = false;
static volatile bool wifi_has_ip = false;
static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;

/* FatFS File System mount descriptor */
static FATFS fat_fs;
static struct fs_mount_t fs_mnt = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
	.mnt_point = "/SD:",
};
static bool sdcard_mounted = false;

/* PRNG helper for simulated sensor readings */
static uint32_t prng_state = 123456789;
static uint32_t prng_range(uint32_t min, uint32_t max)
{
	prng_state = prng_state * 1664525 + 1013904223;
	return min + (prng_state % (max - min + 1));
}

/* ========================================================================= */
/* NTP & Time Formatting Helpers                                             */
/* ========================================================================= */
static void get_formatted_time(char *iso_buf, size_t iso_len, char *date_buf, size_t date_len)
{
	int64_t now_sec;
	k_mutex_lock(&time_mutex, K_FOREVER);
	if (ntp_synced) {
		now_sec = ntp_epoch_sec + (k_uptime_get() - ntp_sync_uptime) / 1000;
	} else {
		/* Fallback starting at 2026-01-01 00:00:00 UTC before NTP sync */
		now_sec = 1767225600LL + (k_uptime_get() / 1000);
	}
	k_mutex_unlock(&time_mutex);

	time_t t = (time_t)now_sec;
	struct tm tm_info;
	gmtime_r(&t, &tm_info);

	int year = (int)(tm_info.tm_year % 10000) + 1900;
	int mon = (int)(tm_info.tm_mon % 12) + 1;
	int day = (int)(tm_info.tm_mday % 32);
	int hour = (int)(tm_info.tm_hour % 24);
	int min = (int)(tm_info.tm_min % 60);
	int sec = (int)(tm_info.tm_sec % 60);

	if (iso_buf && iso_len >= 21) {
		snprintf(iso_buf, iso_len, "%04d-%02d-%02dT%02d:%02d:%02dZ",
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
/* Embedded HTML5 / JS Dashboard with Interactive Hover Tooltip & Crosshairs */
/* ========================================================================= */
static const char HTTP_HTML_PAGE[] =
"<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
"<title>ESP32-S3-GEEK Live Telemetry</title>"
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
"<div class=\"sub\">"
"<span id=\"st\" class=\"badge\">Connecting SSE...</span>"
"<span id=\"ntp\" class=\"badge\">NTP Syncing...</span>"
"<span id=\"sd\" class=\"badge\">SD Card: Active</span>"
"</div>"
"<div class=\"cards\">"
"<div class=\"card\"><div class=\"label\">Motor Speed</div><div class=\"val\" id=\"vRpm\">-- RPM</div></div>"
"<div class=\"card\"><div class=\"label\">Temperature</div><div class=\"val\" id=\"vTmp\">-- &deg;C</div></div>"
"<div class=\"card\"><div class=\"label\">Bus Voltage</div><div class=\"val\" id=\"vVlt\">-- V</div></div>"
"<div class=\"card\"><div class=\"label\">Current</div><div class=\"val\" id=\"vCur\">-- mA</div></div>"
"</div>"
"<div class=\"box\"><div class=\"box-title\">Motor Speed (RPM) [Hover to Inspect]</div><canvas id=\"cRpm\"></canvas></div>"
"<div class=\"box\"><div class=\"box-title\">Sensors (Temp &deg;C: Orange | Bus V: Green) [Hover to Inspect]</div><canvas id=\"cSens\"></canvas></div>"
"<div id=\"tip\"></div>"
"<script>"
"const st=document.getElementById('st');"
"const ntp=document.getElementById('ntp');"
"const tip=document.getElementById('tip');"
"const rpmH=[],tmpH=[],vltH=[],curH=[],timeH=[];"
"const maxPts=3600;"
"let hIdx=-1,hCan=null;"
"function draw(id,dsets,minY,maxY){"
"const c=document.getElementById(id);const ctx=c.getContext('2d');"
"const dpr=window.devicePixelRatio||1;const w=c.clientWidth,h=c.clientHeight;"
"if(c.width!==w*dpr||c.height!==h*dpr){c.width=w*dpr;c.height=h*dpr}"
"ctx.save();ctx.scale(dpr,dpr);ctx.clearRect(0,0,w,h);"
"ctx.strokeStyle='#2a2a2a';ctx.lineWidth=1;"
"for(let y=0;y<=h;y+=h/3){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke()}"
"const n=dsets[0].data.length;"
"if(n<2){ctx.restore();return;}"
"const step=w/(n-1);"
"dsets.forEach(ds=>{"
"ctx.strokeStyle=ds.color;ctx.lineWidth=2;ctx.beginPath();"
"ds.data.forEach((v,i)=>{"
"const x=i*step;"
"const y=h-(Math.max(minY,Math.min(maxY,v))-minY)/(maxY-minY)*(h-16)-8;"
"if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});"
"ctx.stroke();"
"if(hIdx>=0&&hIdx<n){"
"const hx=hIdx*step;"
"const hy=h-(Math.max(minY,Math.min(maxY,ds.data[hIdx]))-minY)/(maxY-minY)*(h-16)-8;"
"ctx.fillStyle=ds.color;ctx.beginPath();ctx.arc(hx,hy,4,0,Math.PI*2);ctx.fill();}"
"});"
"if(hIdx>=0&&hIdx<n){"
"const hx=hIdx*step;"
"ctx.strokeStyle='rgba(255,255,255,0.3)';ctx.setLineDash([4,4]);ctx.beginPath();"
"ctx.moveTo(hx,0);ctx.lineTo(hx,h);ctx.stroke();}"
"ctx.restore();}"
"function updateCharts(){"
"draw('cRpm',[{data:rpmH,color:'#4fc3f7'}],2000,4000);"
"draw('cSens',[{data:tmpH,color:'#ff7043'},{data:vltH,color:'#81c784'}],10,60);}"
"function hookHover(id){"
"const c=document.getElementById(id);"
"function handle(e){"
"const rect=c.getBoundingClientRect();"
"const cx=e.clientX||(e.touches&&e.touches[0].clientX);"
"const cy=e.clientY||(e.touches&&e.touches[0].clientY);"
"if(cx===undefined)return;"
"const x=cx-rect.left;const n=rpmH.length;"
"if(n<2)return;"
"hIdx=Math.round((x/c.clientWidth)*(n-1));"
"if(hIdx<0)hIdx=0;if(hIdx>=n)hIdx=n-1;"
"hCan=id;updateCharts();"
"tip.style.display='block';"
"tip.style.left=Math.min(window.innerWidth-180,Math.max(10,cx+12))+'px';"
"tip.style.top=(cy-65)+'px';"
"tip.innerHTML='<b>'+timeH[hIdx]+'</b><br>'+"
"'<span style=\"color:#4fc3f7\">RPM: '+rpmH[hIdx]+'</span> | '+"
"'<span style=\"color:#ff7043\">'+tmpH[hIdx].toFixed(1)+'&deg;C</span><br>'+"
"'<span style=\"color:#81c784\">'+vltH[hIdx].toFixed(2)+'V</span> | '+"
"'<span style=\"color:#ba68c8\">'+curH[hIdx]+'mA</span>';}"
"c.addEventListener('mousemove',handle);"
"c.addEventListener('touchmove',handle);"
"function leave(){hIdx=-1;hCan=null;tip.style.display='none';updateCharts();}"
"c.addEventListener('mouseleave',leave);"
"c.addEventListener('touchend',leave);}"
"hookHover('cRpm');hookHover('cSens');"
"const es=new EventSource('/events');"
"es.onopen=()=>{st.textContent='SSE Live Connected';st.className='badge ok';};"
"es.onerror=()=>{st.textContent='SSE Reconnecting...';st.className='badge err';};"
"es.onmessage=(e)=>{"
"try{"
"const d=JSON.parse(e.data);"
"if(d.type==='hist_done'){updateCharts();return;}"
"if(d.clients){st.textContent='SSE Live ('+d.clients+' client'+(d.clients>1?'s':'')+')';st.className='badge ok';}"
"if(d.ntp){ntp.textContent='NTP: Synced';ntp.className='badge ok';}"
"document.getElementById('vRpm').textContent=d.rpm+' RPM';"
"document.getElementById('vTmp').textContent=d.temp.toFixed(1)+' \u00b0C';"
"document.getElementById('vVlt').textContent=d.volt.toFixed(2)+' V';"
"document.getElementById('vCur').textContent=d.curr+' mA';"
"rpmH.push(d.rpm);if(rpmH.length>maxPts)rpmH.shift();"
"tmpH.push(d.temp);if(tmpH.length>maxPts)tmpH.shift();"
"vltH.push(d.volt);if(vltH.length>maxPts)vltH.shift();"
"curH.push(d.curr);if(curH.length>maxPts)curH.shift();"
"timeH.push(d.time||'');if(timeH.length>maxPts)timeH.shift();"
"if(!d.hist||rpmH.length%50===0){updateCharts();}"
"}catch(err){console.error(err);}};"
"window.addEventListener('resize',updateCharts);"
"</script></body></html>";

/* ========================================================================= */
/* Hardware Timer ISR Subroutine                                             */
/* ========================================================================= */
static void timer_top_subroutine(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	k_sem_give(&timer_tick_sem);
}

/* ========================================================================= */
/* Thread 1 (High Priority): Motor Speed Sampler (FG Counter & PWM Setup)    */
/* ========================================================================= */
static void motor_speed_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[Thread 1: Speed Sampler] Started (Priority %d, Target Rate: %d Hz)\n",
	       PRIORITY_HIGH, CONFIG_APP_SAMPLE_RATE_HZ);

	/* Initialize Motor FG pulse input GPIO pin */
	bool fg_ready = false;
	if (fg_spec.port && gpio_is_ready_dt(&fg_spec)) {
		if (gpio_pin_configure_dt(&fg_spec, GPIO_INPUT) == 0) {
			gpio_init_callback(&fg_gpio_cb, fg_gpio_isr, BIT(fg_spec.pin));
			if (gpio_add_callback(fg_spec.port, &fg_gpio_cb) == 0) {
				if (gpio_pin_interrupt_configure_dt(&fg_spec, GPIO_INT_EDGE_FALLING) == 0) {
					printk("[Thread 1] Configured FG GPIO interrupt on pin %d\n", fg_spec.pin);
					fg_ready = true;
				}
			}
		}
	}

	if (!fg_ready) {
		printk("[Thread 1] FG pin not available, running in bench simulation mode\n");
	}

	/* Initialize Motor PWM output */
	if (pwm_motor.dev && pwm_is_ready_dt(&pwm_motor)) {
		/* Set default 50% duty cycle at 25 kHz */
		pwm_set_pulse_dt(&pwm_motor, pwm_motor.period / 2);
		printk("[Thread 1] Initialized Motor PWM on pin %d (25 kHz, 50%% duty cycle)\n", 7);
	} else {
		printk("[Thread 1] Motor PWM device not ready or disabled in overlay\n");
	}

	uint32_t last_pulses = 0;

	while (1) {
		k_sem_take(&timer_tick_sem, K_FOREVER);

		motor_speed_sample_t sample;
		sample.timestamp_ms = k_uptime_get();

		if (fg_ready) {
			uint32_t cur_pulses = g_fg_pulse_counter;
			uint32_t delta = cur_pulses - last_pulses;
			last_pulses = cur_pulses;
			sample.pulse_count = delta;
			sample.rpm = (delta * 60 * CONFIG_APP_SAMPLE_RATE_HZ) / CONFIG_APP_MOTOR_PULSES_PER_REV;
		} else {
			/* Bench simulation: ~3000 RPM nominal */
			uint32_t nominal_pulses = (3000 * CONFIG_APP_MOTOR_PULSES_PER_REV) / (60 * CONFIG_APP_SAMPLE_RATE_HZ);
			sample.pulse_count = prng_range(nominal_pulses > 2 ? nominal_pulses - 2 : 1, nominal_pulses + 2);
			sample.rpm = (sample.pulse_count * 60 * CONFIG_APP_SAMPLE_RATE_HZ) / CONFIG_APP_MOTOR_PULSES_PER_REV;
		}

		k_msgq_put(&motor_speed_msgq, &sample, K_NO_WAIT);
	}
}

/* ========================================================================= */
/* Thread 2 (Medium Priority): Sensor Aggregator (INA219 I2C & Time Tagger)  */
/* ========================================================================= */
static void sensor_aggregator_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint32_t sequence = 0;
	motor_speed_sample_t speed_sample;

	printk("[Thread 2: Aggregator] Started (Priority %d)\n", PRIORITY_MEDIUM);

	if (ina219_dev && device_is_ready(ina219_dev)) {
		printk("[Thread 2] INA219 I2C sensor ready on bus %s\n", ina219_dev->name);
	} else {
		printk("[Thread 2] INA219 not detected, running with simulated bus voltage & current\n");
	}

	while (1) {
		k_msgq_get(&motor_speed_msgq, &speed_sample, K_FOREVER);
		sequence++;

		telemetry_packet_t packet;
		packet.sample_id = sequence;
		packet.motor_rpm = speed_sample.rpm;
		packet.uptime_ms = speed_sample.timestamp_ms;

		bool ina_ok = false;
		if (ina219_dev && device_is_ready(ina219_dev)) {
			if (sensor_sample_fetch(ina219_dev) == 0) {
				struct sensor_value v_bus, i_curr;
				sensor_channel_get(ina219_dev, SENSOR_CHAN_VOLTAGE, &v_bus);
				sensor_channel_get(ina219_dev, SENSOR_CHAN_CURRENT, &i_curr);

				packet.bus_mv = (uint32_t)(v_bus.val1 * 1000 + v_bus.val2 / 1000);
				packet.current_ma = (uint32_t)(i_curr.val1 * 1000 + i_curr.val2 / 1000);
				ina_ok = true;
			}
		}

		if (!ina_ok) {
			/* Simulation fallback */
			packet.bus_mv     = prng_range(11850, 12150);
			packet.current_ma = prng_range(780, 920);
		}
		packet.temp_mc    = (int32_t)prng_range(41200, 42800);

		get_formatted_time(packet.iso_time, sizeof(packet.iso_time), NULL, 0);

		/* Update snapshot for live SSE broadcasting */
		k_mutex_lock(&telemetry_mutex, K_FOREVER);
		latest_telemetry = packet;
		has_telemetry = true;
		k_condvar_broadcast(&telemetry_cv);
		k_mutex_unlock(&telemetry_mutex);

		/* Enqueue packet to Thread 3 for SD card CSV logging */
		k_msgq_put(&csv_msgq, &packet, K_NO_WAIT);
	}
}

/* ========================================================================= */
/* Thread 3 (Low Priority): SD Card FAT32 CSV Logger & Daily Auto-Rotator    */
/* ========================================================================= */
static int init_sdcard_fat32(void)
{
	int res = fs_mount(&fs_mnt);
	if (res != 0) {
		printk("[SD Card] Initial mount failed (err %d). Formatting as FAT32...\n", res);
		res = fs_mkfs(FS_FATFS, (uintptr_t)"SD:", NULL, 0);
		if (res != 0) {
			printk("[SD Card] FAT32 format failed: %d\n", res);
			return res;
		}
		printk("[SD Card] Format complete! Mounting volume...\n");
		res = fs_mount(&fs_mnt);
		if (res != 0) {
			printk("[SD Card] Re-mount failed after format: %d\n", res);
			return res;
		}
	}

	printk("[SD Card] FAT32 Volume mounted successfully at %s\n", fs_mnt.mnt_point);
	sdcard_mounted = true;
	return 0;
}

static void sdcard_csv_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[Thread 3: SD CSV Logger] Started (Priority %d)\n", PRIORITY_LOW);

	/* Initialize SD card and format as FAT32 if necessary */
	init_sdcard_fat32();

	char current_day_str[24] = "";
	struct fs_file_t csv_file;
	fs_file_t_init(&csv_file);
	bool file_is_open = false;

	while (1) {
		telemetry_packet_t packet;
		k_msgq_get(&csv_msgq, &packet, K_FOREVER);

		if (!sdcard_mounted) {
			/* Retry mounting if card wasn't available at boot */
			if (init_sdcard_fat32() != 0) {
				continue;
			}
		}

		char day_str[24];
		get_formatted_time(NULL, 0, day_str, sizeof(day_str));

		/* Daily file rotation: If day changed or first run */
		if (strcmp(day_str, current_day_str) != 0) {
			if (file_is_open) {
				fs_close(&csv_file);
				file_is_open = false;
			}

			strncpy(current_day_str, day_str, sizeof(current_day_str));
			char filepath[64];
			snprintf(filepath, sizeof(filepath), "/SD:/telemetry_%s.csv", current_day_str);

			struct fs_dirent entry;
			bool exists = (fs_stat(filepath, &entry) == 0);

			int res = fs_open(&csv_file, filepath, FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
			if (res == 0) {
				file_is_open = true;

				/* Write CSV header if creating a new file */
				if (!exists || entry.size == 0) {
					const char *hdr = "timestamp_iso,uptime_ms,sample_id,rpm,temp_c,bus_volt_v,curr_ma\n";
					fs_write(&csv_file, hdr, strlen(hdr));
					fs_sync(&csv_file);
				}

				k_mutex_lock(&csv_path_mutex, K_FOREVER);
				strncpy(active_csv_filename, filepath, sizeof(active_csv_filename));
				k_mutex_unlock(&csv_path_mutex);

				printk("[SD Card] Opened/Rotated CSV log: %s\n", filepath);
			} else {
				printk("[SD Card] Failed to open CSV file %s (err %d)\n", filepath, res);
			}
		}

		/* Append telemetry record */
		if (file_is_open) {
			char line[128];
			int len = snprintf(line, sizeof(line), "%s,%lld,%u,%u,%d.%d,%u.%02u,%u\n",
			                   packet.iso_time,
			                   packet.uptime_ms,
			                   packet.sample_id,
			                   packet.motor_rpm,
			                   packet.temp_mc / 1000, abs(packet.temp_mc % 1000) / 100,
			                   packet.bus_mv / 1000, (packet.bus_mv % 1000) / 10,
			                   packet.current_ma);

			if (len > 0) {
				fs_write(&csv_file, line, len);
				/* Flush to card periodically (~once per second) to preserve SD flash endurance */
				if (packet.sample_id % CONFIG_APP_SAMPLE_RATE_HZ == 0) {
					fs_sync(&csv_file);
				}
			}
		}
	}
}

/* ========================================================================= */
/* Replay Recent History from CSV to newly connected SSE Client (Max 1 hr)    */
/* ========================================================================= */
static void replay_csv_history_to_sse(int client_fd)
{
	char filepath[64] = "";
	k_mutex_lock(&csv_path_mutex, K_FOREVER);
	if (strlen(active_csv_filename) > 0) {
		strncpy(filepath, active_csv_filename, sizeof(filepath));
	}
	k_mutex_unlock(&csv_path_mutex);

	if (strlen(filepath) == 0) {
		return;
	}

	struct fs_dirent entry;
	if (fs_stat(filepath, &entry) != 0 || entry.size == 0) {
		return;
	}

	struct fs_file_t rf;
	fs_file_t_init(&rf);
	if (fs_open(&rf, filepath, FS_O_READ) != 0) {
		return;
	}

	/* Max 1 hour at 1 Hz is ~3600 samples * ~60 bytes/sample = ~216 KB */
	const off_t max_bytes = MAX_HIST_REPLAY_SAMPLES * 65;
	if (entry.size > max_bytes) {
		fs_seek(&rf, entry.size - max_bytes, FS_SEEK_SET);
		/* Discard partial line until next newline */
		char c;
		while (fs_read(&rf, &c, 1) > 0 && c != '\n');
	} else {
		/* Skip CSV header */
		char c;
		while (fs_read(&rf, &c, 1) > 0 && c != '\n');
	}

	/* Read lines and pump historical SSE records */
	char line_buf[128];
	size_t idx = 0;
	char ch;
	int sent_count = 0;

	while (fs_read(&rf, &ch, 1) > 0) {
		if (ch == '\n' || ch == '\r') {
			if (idx > 0) {
				line_buf[idx] = '\0';
				char t_iso[24];
				long long up;
				unsigned int sid, rpm, curr;
				int tc_int, tc_fr;
				unsigned int v_int, v_fr;

				if (sscanf(line_buf, "%23[^,],%lld,%u,%u,%d.%d,%u.%u,%u",
				           t_iso, &up, &sid, &rpm, &tc_int, &tc_fr, &v_int, &v_fr, &curr) >= 8) {
					char sse_item[220];
					int slen = snprintf(sse_item, sizeof(sse_item),
					                    "data: {\"id\":%u,\"rpm\":%u,\"temp\":%d.%d,\"volt\":%u.%02u,\"curr\":%u,\"uptime\":%lld,\"time\":\"%s\",\"hist\":true}\n\n",
					                    sid, rpm, tc_int, tc_fr, v_int, v_fr, curr, up, t_iso);
					if (send_all(client_fd, sse_item, slen) < 0) {
						break;
					}
					sent_count++;
				}
				idx = 0;
			}
		} else {
			if (idx < sizeof(line_buf) - 1) {
				line_buf[idx++] = ch;
			}
		}
	}

	fs_close(&rf);

	/* Signal history load complete */
	const char *hist_done = "data: {\"type\":\"hist_done\"}\n\n";
	send_all(client_fd, hist_done, strlen(hist_done));
	printk("[HTTP] Replayed %d historical CSV records to SSE client\n", sent_count);
}

/* ========================================================================= */
/* Thread 4 (Wi-Fi, Auto-Reconnect & HTTP/SSE Web Server)                    */
/* ========================================================================= */
static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
                                    uint64_t mgmt_event, struct net_if *iface)
{
	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT: {
		const struct wifi_status *status = (const struct wifi_status *)cb->info;
		if (status->status == 0) {
			printk("[Wi-Fi] Connected to SSID: %s\n", CONFIG_APP_WIFI_SSID);
			wifi_connected = true;
		} else {
			printk("[Wi-Fi] Connection attempt failed (status %d), will retry...\n", status->status);
			wifi_connected = false;
			wifi_has_ip = false;
		}
		break;
	}
	case NET_EVENT_WIFI_DISCONNECT_RESULT: {
		printk("[Wi-Fi] Disconnected from network! Auto-reconnect active...\n");
		wifi_connected = false;
		wifi_has_ip = false;
		break;
	}
	case NET_EVENT_IPV4_ADDR_ADD: {
		struct net_if_config *cfg = net_if_get_config(iface);
		if (!cfg || !cfg->ip.ipv4) {
			break;
		}
		char buf[NET_IPV4_ADDR_LEN];
		for (int i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
			if (cfg->ip.ipv4->unicast[i].ipv4.addr_type == NET_ADDR_DHCP ||
			    cfg->ip.ipv4->unicast[i].ipv4.is_used) {
				net_addr_ntop(AF_INET, &cfg->ip.ipv4->unicast[i].ipv4.address.in_addr,
				              buf, sizeof(buf));
				printk("\n======================================================\n");
				printk(">>> ESP32-S3-GEEK Web Dashboard is LIVE! <<<\n");
				printk(">>> Open browser at: http://%s/ <<<\n", buf);
				printk("======================================================\n\n");
				wifi_has_ip = true;
				k_sem_give(&ntp_trigger_sem);
				break;
			}
		}
		break;
	}
	case NET_EVENT_IPV4_ADDR_DEL: {
		printk("[Wi-Fi] IPv4 address released\n");
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
	struct wifi_connect_req_params params = {
		.ssid = CONFIG_APP_WIFI_SSID,
		.ssid_length = strlen(CONFIG_APP_WIFI_SSID),
		.psk = CONFIG_APP_WIFI_PSK,
		.psk_length = strlen(CONFIG_APP_WIFI_PSK),
		.channel = 0,
		.security = strlen(CONFIG_APP_WIFI_PSK) ? WIFI_SECURITY_TYPE_PSK : WIFI_SECURITY_TYPE_NONE,
	};

	printk("[Wi-Fi] Requesting connection to '%s'...\n", CONFIG_APP_WIFI_SSID);
	net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
}

static void setup_wifi_callbacks(void)
{
	net_mgmt_init_event_callback(&wifi_cb, wifi_mgmt_event_handler,
	                             NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_cb);

	net_mgmt_init_event_callback(&ipv4_cb, wifi_mgmt_event_handler,
	                             NET_EVENT_IPV4_ADDR_ADD | NET_EVENT_IPV4_ADDR_DEL);
	net_mgmt_add_event_callback(&ipv4_cb);
}

/* ========================================================================= */
/* Thread 5 (Lowest Priority): Background NTP Time Synchronizer              */
/* ========================================================================= */
static void ntp_sync_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[Thread 5: Background NTP] Started (Priority %d)\n", PRIORITY_NTP);

	while (1) {
		/* If already synchronized, sleep for 1 hour before routine re-sync.
		 * If not synchronized yet, wait up to 10 seconds or until Wi-Fi connects. */
		k_timeout_t wait_timeout = ntp_synced ? K_HOURS(1) : K_SECONDS(10);
		k_sem_take(&ntp_trigger_sem, wait_timeout);

		if (!wifi_has_ip) {
			continue;
		}

		printk("[NTP Background] Querying pool.ntp.org...\n");
		struct sntp_time sntp_res;
		int ret = sntp_simple("pool.ntp.org", 4000, &sntp_res);
		if (ret < 0) {
			printk("[NTP Background] pool.ntp.org failed (%d), retrying with time.google.com...\n", ret);
			ret = sntp_simple("time.google.com", 4000, &sntp_res);
		}

		if (ret == 0) {
			k_mutex_lock(&time_mutex, K_FOREVER);
			ntp_epoch_sec = (int64_t)sntp_res.seconds;
			ntp_sync_uptime = k_uptime_get();
			ntp_synced = true;
			k_mutex_unlock(&time_mutex);
			printk("[NTP Background] Time synchronized successfully! Epoch: %lld (next sync in 1h)\n",
			       (long long)ntp_epoch_sec);
		} else {
			printk("[NTP Background] Query failed (err %d), will retry in 15s...\n", ret);
			k_sleep(K_SECONDS(15));
		}
	}
}

static void wifi_http_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[Thread 4: Wi-Fi & Web Server] Started (Priority %d)\n", PRIORITY_WIFI);

	setup_wifi_callbacks();
	trigger_wifi_connect();

	int server_fd = -1;
	sse_client_t sse_clients[MAX_SSE_CLIENTS];
	for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
		sse_clients[i].fd = -1;
		sse_clients[i].last_sent_id = 0;
	}

	int64_t last_reconnect_attempt = k_uptime_get();

	while (1) {
		int64_t now = k_uptime_get();

		/* Auto-reconnect supervisor: If Wi-Fi is lost, retry every 5 seconds */
		if (!wifi_connected && (now - last_reconnect_attempt > 5000)) {
			last_reconnect_attempt = now;
			trigger_wifi_connect();
		}

		/* If Wi-Fi is disconnected or IP not assigned, clean up dead SSE client sockets */
		if (!wifi_has_ip) {
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
				printk("[HTTP Server] Socket creation failed: %d\n", errno);
				k_sleep(K_MSEC(500));
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
				printk("[HTTP Server] Bind failed on port %d: %d\n", HTTP_PORT, errno);
				close(server_fd);
				server_fd = -1;
				k_sleep(K_MSEC(1000));
				continue;
			}

			if (listen(server_fd, 5) < 0) {
				printk("[HTTP Server] Listen failed: %d\n", errno);
				close(server_fd);
				server_fd = -1;
				k_sleep(K_MSEC(1000));
				continue;
			}

			printk("[HTTP Server] Listening on port %d (supports %d simultaneous SSE clients)...\n",
			       HTTP_PORT, MAX_SSE_CLIENTS);
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
				char rx_buf[256];
				int rlen = recv(client_fd, rx_buf, sizeof(rx_buf) - 1, 0);
				if (rlen > 0) {
					rx_buf[rlen] = '\0';

					/* Match GET /events (Server-Sent Events) */
					if (strncmp(rx_buf, "GET /events", 11) == 0) {
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
								/* Stream recent CSV history before entering live mode */
								replay_csv_history_to_sse(client_fd);

								sse_clients[slot].fd = client_fd;
								sse_clients[slot].last_sent_id = 0;
								printk("[HTTP] SSE client #%d connected on fd %d\n",
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
		telemetry_packet_t cur;
		bool has_new_packet = false;

		k_mutex_lock(&telemetry_mutex, K_FOREVER);
		if (has_telemetry) {
			cur = latest_telemetry;
			has_new_packet = true;
		}
		k_mutex_unlock(&telemetry_mutex);

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

				char sse_data[240];
				int sse_len = snprintf(sse_data, sizeof(sse_data),
				                       "data: {\"id\":%u,\"rpm\":%u,\"temp\":%d.%d,\"volt\":%u.%02u,\"curr\":%u,\"uptime\":%lld,\"time\":\"%s\",\"clients\":%d,\"ntp\":%s}\n\n",
				                       cur.sample_id,
				                       cur.motor_rpm,
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
						if (send_all(sse_clients[i].fd, sse_data, sse_len) < 0) {
							printk("[HTTP] SSE client #%d write failed, closing\n", i + 1);
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
}

/* ========================================================================= */
/* Static Thread Definitions                                                 */
/* ========================================================================= */
K_THREAD_DEFINE(thread_speed, THREAD_STACK_SIZE, motor_speed_thread_fn,
                NULL, NULL, NULL, PRIORITY_HIGH, 0, 0);

K_THREAD_DEFINE(thread_aggregator, THREAD_STACK_SIZE, sensor_aggregator_thread_fn,
                NULL, NULL, NULL, PRIORITY_MEDIUM, 0, 0);

K_THREAD_DEFINE(thread_csv, CSV_STACK_SIZE, sdcard_csv_thread_fn,
                NULL, NULL, NULL, PRIORITY_LOW, 0, 0);

K_THREAD_DEFINE(thread_wifi_http, WIFI_STACK_SIZE, wifi_http_thread_fn,
                NULL, NULL, NULL, PRIORITY_WIFI, 0, 0);

K_THREAD_DEFINE(thread_ntp, NTP_STACK_SIZE, ntp_sync_thread_fn,
                NULL, NULL, NULL, PRIORITY_NTP, 0, 0);

/* ========================================================================= */
/* Main Application Entry Point                                              */
/* ========================================================================= */
int main(void)
{
	const struct device *const timer_dev = DEVICE_DT_GET(TIMER_NODE);
	int err;

	printk("=================================================================\n");
	printk("Waveshare ESP32-S3-GEEK: Multi-Threaded Sensor & SD Web Server\n");
	printk("Architecture: Xtensa Dual-Core ESP32-S3 (PROCPU Primary)\n");
	printk("HW Timer: %s (%s)\n", timer_dev->name, DT_NODE_PATH(TIMER_NODE));
	printk("=================================================================\n");

	if (!device_is_ready(timer_dev)) {
		printk("FATAL: Counter device %s is not ready!\n", timer_dev->name);
		return -ENODEV;
	}

	uint32_t freq = counter_get_frequency(timer_dev);
	printk("Timer frequency: %u Hz\n", freq);

	/* Configure periodic top value (1~10 Hz auto-reload) */
	struct counter_top_cfg top_cfg = {
		.ticks = counter_us_to_ticks(timer_dev, TIMER_INTERVAL_US),
		.callback = timer_top_subroutine,
		.user_data = NULL,
		.flags = 0,
	};

	printk("Starting hardware timer: period = %d us (%d Hz, %llu ticks)...\n",
	       TIMER_INTERVAL_US, CONFIG_APP_SAMPLE_RATE_HZ,
	       (unsigned long long)top_cfg.ticks);

	err = counter_set_top_value(timer_dev, &top_cfg);
	if (err) {
		printk("FATAL: Failed to set counter top value: %d\n", err);
		return err;
	}

	err = counter_start(timer_dev);
	if (err) {
		printk("FATAL: Failed to start counter: %d\n", err);
		return err;
	}

	printk("Hardware timer running. Pipeline, SD logger, & Wi-Fi Server active.\n\n");

	while (1) {
		k_sleep(K_SECONDS(30));
		printk("[Supervisor] System healthy, 64-bit uptime: %lld ms | Wi-Fi: %s | NTP: %s\n",
		       k_uptime_get(),
		       wifi_has_ip ? "Connected" : (wifi_connected ? "Associated (No IP)" : "Disconnected"),
		       ntp_synced ? "Synced" : "Pending");
	}

	return 0;
}
