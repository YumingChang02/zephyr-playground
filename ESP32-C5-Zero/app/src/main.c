/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Waveshare ESP32-C5-Zero: Multi-Threaded Sensor Pipeline with Wi-Fi & Web Server
 *
 * Thread Architecture:
 *   [HW Timer ISR] -> signals sem -> [Thread 1: Speed Sampler (High Prio)]
 *                                          │ (motor_speed_msgq)
 *                                          ▼
 *                                     [Thread 2: Sensor Aggregator (Med Prio)]
 *                                     ├── (telemetry_msgq) -> [Thread 3: Telemetry Logger (Low Prio)]
 *                                     └── (condvar/mutex)  -> [Thread 4: Wi-Fi HTTP / SSE Server (Background)]
 *
 * Web Endpoints:
 *   GET /        -> Self-contained Interactive HTML5 Canvas Dashboard (Zero CDN dependency)
 *   GET /events  -> Server-Sent Events (SSE) streaming live JSON telemetry
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/socket.h>

/* POSIX socket convenience aliases for Zephyr zsock_* */
#define socket(f, t, p)      zsock_socket(f, t, p)
#define bind(s, a, l)        zsock_bind(s, a, l)
#define listen(s, b)         zsock_listen(s, b)
#define accept(s, a, l)      zsock_accept(s, a, l)
#define send(s, b, l, f)     zsock_send(s, b, l, f)
#define recv(s, b, l, f)     zsock_recv(s, b, l, f)
#define poll(p, n, t)        zsock_poll(p, n, t)
#define setsockopt(s, l, o, v, len) zsock_setsockopt(s, l, o, v, len)
#define close(s)             zsock_close(s)

/* ========================================================================= */
/* Wi-Fi Configuration (Edit these or pass via prj.conf)                     */
/* ========================================================================= */
#define HTTP_PORT            80
#define MAX_SSE_CLIENTS      4

/* SSE Client Session Structure */
typedef struct {
	int fd;
	uint32_t last_sent_id;
} sse_client_t;

/* ========================================================================= */
/* Thread Priorities and Stack Sizes                                         */
/* Higher number = LOWER priority in Zephyr preemptive scheduling            */
/* ========================================================================= */
#define PRIORITY_HIGH        3   /* Thread 1: Motor Speed Sampler            */
#define PRIORITY_MEDIUM      5   /* Thread 2: Sensor Aggregator              */
#define PRIORITY_LOW         7   /* Thread 3: Background Telemetry Logger    */
#define PRIORITY_WIFI        8   /* Thread 4: Wi-Fi & Web Server             */

#define THREAD_STACK_SIZE    2048
#define WIFI_STACK_SIZE      4096

/* Hardware timer interval: 1.0 second (1,000,000 us) */
#define TIMER_INTERVAL_US    1000000

/* Devicetree hardware timer alias */
#define TIMER_NODE DT_ALIAS(app_timer)

BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(TIMER_NODE),
             "Devicetree error: 'app-timer' alias is missing or disabled!");

/* ========================================================================= */
/* Data Structures & Message Queues                                          */
/* ========================================================================= */

/* Data produced by Thread 1: Motor FG pulse counter and calculated RPM */
typedef struct {
	uint32_t pulse_count;   /* Raw FG pulses captured in sampling window */
	uint32_t rpm;           /* Calculated motor RPM                      */
	int64_t  timestamp_ms;  /* Sampling timestamp                        */
} motor_speed_sample_t;

/* Aggregated data produced by Thread 2: Motor + Voltage/Current + Temp */
typedef struct {
	uint32_t sample_id;     /* Monotonic sequence number                 */
	uint32_t motor_rpm;     /* Motor speed in RPM                        */
	int32_t  temp_mc;       /* Motor Temperature in milli-Celsius (e.g. 41500 = 41.5 C) */
	uint32_t bus_mv;        /* INA219 Bus Voltage in milli-Volts (e.g. 12100 = 12.1 V)  */
	uint32_t current_ma;    /* INA219 Motor Current in milli-Amps (e.g. 850 = 0.85 A)   */
	int64_t  uptime_ms;     /* 64-bit non-rolling uptime                 */
} telemetry_packet_t;

/* Message Queue 1: Transfers motor speed from Thread 1 -> Thread 2 */
K_MSGQ_DEFINE(motor_speed_msgq, sizeof(motor_speed_sample_t), 4, 4);

/* Message Queue 2: Transfers complete telemetry from Thread 2 -> Thread 3 */
K_MSGQ_DEFINE(telemetry_msgq, sizeof(telemetry_packet_t), 4, 4);

/* Semaphore: Hardware timer ISR subroutine -> Thread 1 */
static K_SEM_DEFINE(timer_tick_sem, 0, 1);

/* Latest Telemetry snapshot shared with Thread 4 (HTTP / SSE web server) */
static K_MUTEX_DEFINE(telemetry_mutex);
static K_CONDVAR_DEFINE(telemetry_cv);
static telemetry_packet_t latest_telemetry;
static bool has_telemetry = false;

/* Wi-Fi Connection state */
static volatile bool wifi_connected = false;
static volatile bool wifi_has_ip = false;
static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;

/* Simple PRNG for simulating realistic hardware variation */
static uint32_t prng_state = 123456789;
static uint32_t prng_range(uint32_t min, uint32_t max)
{
	prng_state = prng_state * 1664525 + 1013904223;
	return min + (prng_state % (max - min + 1));
}

/* ========================================================================= */
/* Embedded HTML / JS Dashboard (Self-contained, Zero-CDN Canvas Charts)      */
/* ========================================================================= */
static const char HTTP_HTML_PAGE[] =
"<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
"<title>ESP32-C5 Live Telemetry</title>"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<style>"
"body{font-family:system-ui,-apple-system,sans-serif;background:#121212;color:#e0e0e0;margin:0;padding:16px}"
"h1{font-size:1.3rem;color:#4fc3f7;margin:0 0 8px 0}"
".badge{display:inline-block;padding:4px 10px;border-radius:4px;font-size:0.8rem;margin-bottom:12px;background:#333;color:#aaa}"
".badge.ok{background:#1b5e20;color:#81c784}"
".badge.err{background:#b71c1c;color:#ef9a9a}"
".cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:10px;margin-bottom:14px}"
".card{background:#1e1e1e;border:1px solid #333;border-radius:6px;padding:10px}"
".label{font-size:0.7rem;color:#9e9e9e;text-transform:uppercase}"
".val{font-size:1.3rem;font-weight:700;margin-top:2px}"
".box{background:#1e1e1e;border:1px solid #333;border-radius:6px;padding:10px;margin-bottom:12px}"
".box-title{font-size:0.8rem;color:#bbb;margin-bottom:6px}"
"canvas{width:100%;height:100px;display:block}"
"</style></head><body>"
"<h1>ESP32-C5 Telemetry Dashboard</h1>"
"<div id=\"st\" class=\"badge\">Connecting SSE...</div>"
"<div class=\"cards\">"
"<div class=\"card\"><div class=\"label\">Motor Speed</div><div class=\"val\" id=\"vRpm\">-- RPM</div></div>"
"<div class=\"card\"><div class=\"label\">Temperature</div><div class=\"val\" id=\"vTmp\">-- &deg;C</div></div>"
"<div class=\"card\"><div class=\"label\">Bus Voltage</div><div class=\"val\" id=\"vVlt\">-- V</div></div>"
"<div class=\"card\"><div class=\"label\">Current</div><div class=\"val\" id=\"vCur\">-- mA</div></div>"
"</div>"
"<div class=\"box\"><div class=\"box-title\">Motor Speed (RPM)</div><canvas id=\"cRpm\"></canvas></div>"
"<div class=\"box\"><div class=\"box-title\">Sensors (Temp &deg;C: Orange | Bus V: Green)</div><canvas id=\"cSens\"></canvas></div>"
"<script>"
"const st=document.getElementById('st');"
"const rpmH=[],tmpH=[],vltH=[];const maxPts=30;"
"function draw(id,dsets,minY,maxY){"
"const c=document.getElementById(id);const ctx=c.getContext('2d');"
"const dpr=window.devicePixelRatio||1;const w=c.clientWidth,h=c.clientHeight;"
"if(c.width!==w*dpr||c.height!==h*dpr){c.width=w*dpr;c.height=h*dpr}"
"ctx.save();ctx.scale(dpr,dpr);ctx.clearRect(0,0,w,h);"
"ctx.strokeStyle='#2a2a2a';ctx.lineWidth=1;"
"for(let y=0;y<=h;y+=h/3){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke()}"
"dsets.forEach(ds=>{"
"if(ds.data.length<2)return;"
"ctx.strokeStyle=ds.color;ctx.lineWidth=2;ctx.beginPath();"
"const step=w/(maxPts-1);const startX=w-(ds.data.length-1)*step;"
"ds.data.forEach((v,i)=>{"
"const x=startX+i*step;"
"const y=h-(Math.max(minY,Math.min(maxY,v))-minY)/(maxY-minY)*(h-12)-6;"
"if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});"
"ctx.stroke();});ctx.restore();}"
"function updateCharts(){"
"draw('cRpm',[{data:rpmH,color:'#4fc3f7'}],2000,4000);"
"draw('cSens',[{data:tmpH,color:'#ff7043'},{data:vltH,color:'#81c784'}],10,60);}"
"const es=new EventSource('/events');"
"es.onopen=()=>{st.textContent='SSE Live Connected';st.className='badge ok';};"
"es.onerror=()=>{st.textContent='SSE Reconnecting...';st.className='badge err';};"
"es.onmessage=(e)=>{"
"try{"
"const d=JSON.parse(e.data);"
"if(d.clients){st.textContent='SSE Live ('+d.clients+' client'+(d.clients>1?'s':'')+')';st.className='badge ok';}"
"document.getElementById('vRpm').textContent=d.rpm+' RPM';"
"document.getElementById('vTmp').textContent=d.temp.toFixed(1)+' \u00b0C';"
"document.getElementById('vVlt').textContent=d.volt.toFixed(2)+' V';"
"document.getElementById('vCur').textContent=d.curr+' mA';"
"rpmH.push(d.rpm);if(rpmH.length>maxPts)rpmH.shift();"
"tmpH.push(d.temp);if(tmpH.length>maxPts)tmpH.shift();"
"vltH.push(d.volt);if(vltH.length>maxPts)vltH.shift();"
"updateCharts();"
"}catch(err){console.error(err);}};"
"window.addEventListener('resize',updateCharts);"
"</script></body></html>";

/* Helper: Reliably send all bytes across TCP, retrying partial sends */
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
/* Hardware Timer ISR Subroutine                                             */
/* ========================================================================= */
static void timer_top_subroutine(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	/* Unblock High-Priority Thread 1 */
	k_sem_give(&timer_tick_sem);
}

/* ========================================================================= */
/* Thread 1 (High Priority): Motor FG Speed Sampler                          */
/* ========================================================================= */
static void motor_speed_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[Thread 1: Speed Sampler] Started (Priority %d)\n", PRIORITY_HIGH);

	while (1) {
		/* Wait for periodic hardware timer trigger */
		k_sem_take(&timer_tick_sem, K_FOREVER);

		/*
		 * Simulation: Measure Frequency Generator (FG) pulses from BLDC motor.
		 * Typically a BLDC motor has 3 pole pairs -> 6 pulses per mechanical revolution.
		 * Over 1 second: 300 pulses = 300 / 6 * 60 = 3000 RPM.
		 */
		motor_speed_sample_t sample;
		sample.pulse_count = prng_range(280, 320); /* Simulated pulse count */
		sample.rpm = (sample.pulse_count * 60) / 6;
		sample.timestamp_ms = k_uptime_get();

		/* Send raw speed sample to Thread 2 via Message Queue */
		k_msgq_put(&motor_speed_msgq, &sample, K_NO_WAIT);
	}
}

/* ========================================================================= */
/* Thread 2 (Medium Priority): Sensor Aggregator & Controller                */
/* ========================================================================= */
static void sensor_aggregator_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint32_t sequence = 0;
	motor_speed_sample_t speed_sample;

	printk("[Thread 2: Aggregator] Started (Priority %d)\n", PRIORITY_MEDIUM);

	while (1) {
		/* Block until Thread 1 pushes a motor speed sample */
		k_msgq_get(&motor_speed_msgq, &speed_sample, K_FOREVER);
		sequence++;

		/*
		 * Read secondary sensors (simulated for now, replaced with DS18B20
		 * and INA219 drivers on target hardware):
		 * - DS18B20 temperature: ~41.5°C to ~43.0°C (milli-Celsius)
		 * - INA219 voltage: ~11,800 mV to ~12,200 mV (nominal 12V bus)
		 * - INA219 current: ~750 mA to ~950 mA
		 */
		telemetry_packet_t packet;
		packet.sample_id = sequence;
		packet.motor_rpm = speed_sample.rpm;
		packet.uptime_ms = speed_sample.timestamp_ms;
		packet.temp_mc    = (int32_t)prng_range(41200, 42800);
		packet.bus_mv     = prng_range(11850, 12150);
		packet.current_ma = prng_range(780, 920);

		/* Update latest telemetry snapshot and broadcast to Wi-Fi SSE thread */
		k_mutex_lock(&telemetry_mutex, K_FOREVER);
		latest_telemetry = packet;
		has_telemetry = true;
		k_condvar_broadcast(&telemetry_cv);
		k_mutex_unlock(&telemetry_mutex);

		/* Send complete telemetry packet to Thread 3 for background logging */
		k_msgq_put(&telemetry_msgq, &packet, K_NO_WAIT);
	}
}

/* ========================================================================= */
/* Thread 3 (Low Priority): Background Logger & Telemetry Consumer           */
/* ========================================================================= */
static void telemetry_logger_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	telemetry_packet_t packet;

	printk("[Thread 3: Logger] Started (Priority %d)\n", PRIORITY_LOW);

	while (1) {
		/* Block until Thread 2 pushes an aggregated packet */
		k_msgq_get(&telemetry_msgq, &packet, K_FOREVER);

		/*
		 * In the target system, this thread writes to MicroSD card and
		 * updates the Wi-Fi HTTP buffer. Here we log structured telemetry:
		 */
		int32_t temp_c_int  = packet.temp_mc / 1000;
		int32_t temp_c_frac = (packet.temp_mc % 1000) / 100;
		uint32_t volt_int   = packet.bus_mv / 1000;
		uint32_t volt_frac  = (packet.bus_mv % 1000) / 100;

		printk("[TELEMETRY #%04u | %lld ms] RPM: %4u | Temp: %d.%d C | Bus: %u.%u V | Curr: %3u mA\n",
		       packet.sample_id, packet.uptime_ms, packet.motor_rpm,
		       temp_c_int, temp_c_frac, volt_int, volt_frac, packet.current_ma);
	}
}

/* ========================================================================= */
/* Thread 4 (Wi-Fi & HTTP / SSE Server)                                      */
/* ========================================================================= */

/* Network management event callback: Handles Wi-Fi status and DHCP IPv4 address */
static void handle_wifi_mgmt_event(struct net_mgmt_event_callback *cb,
                                   uint64_t mgmt_event, struct net_if *iface)
{
	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT: {
		const struct wifi_status *status = (const struct wifi_status *)cb->info;
		if (status->status == 0) {
			printk("[Wi-Fi] Connected successfully to %s\n", CONFIG_APP_WIFI_SSID);
			wifi_connected = true;
		} else {
			printk("[Wi-Fi] Connection failed with status: %d, will retry...\n", status->status);
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
				net_addr_ntop(AF_INET,
				              &cfg->ip.ipv4->unicast[i].ipv4.address.in_addr,
				              buf, sizeof(buf));
				printk("\n======================================================\n");
				printk(">>> ESP32-C5 Web Dashboard is LIVE! <<<\n");
				printk(">>> Open browser at: http://%s/ <<<\n", buf);
				printk("======================================================\n\n");
				wifi_has_ip = true;
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

	printk("[Wi-Fi] Connecting to SSID: '%s'...\n", CONFIG_APP_WIFI_SSID);
	int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
	if (ret != 0) {
		printk("[Wi-Fi] Connect request returned: %d\n", ret);
	}
}

/* Initialize Wi-Fi station mode and register callbacks */
static void start_wifi_station(void)
{
	/* Register Wi-Fi event callbacks */
	net_mgmt_init_event_callback(&wifi_cb, handle_wifi_mgmt_event,
	                             NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_cb);

	net_mgmt_init_event_callback(&ipv4_cb, handle_wifi_mgmt_event,
	                             NET_EVENT_IPV4_ADDR_ADD | NET_EVENT_IPV4_ADDR_DEL);
	net_mgmt_add_event_callback(&ipv4_cb);

	trigger_wifi_connect();
}

/* Thread 4 Entry: Non-blocking HTTP & SSE server with auto-reconnect */
static void wifi_http_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[Thread 4: Wi-Fi & Web Server] Started (Priority %d)\n", PRIORITY_WIFI);

	/* Initialize and start Wi-Fi station */
	start_wifi_station();

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

		/* If disconnected or IP lost, clean up dead client sockets */
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
				printk("[HTTP Server] Failed to create socket: %d\n", errno);
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
				printk("[HTTP Server] Failed to bind to port %d (errno %d)\n", HTTP_PORT, errno);
				close(server_fd);
				server_fd = -1;
				k_sleep(K_MSEC(1000));
				continue;
			}

			if (listen(server_fd, 5) < 0) {
				printk("[HTTP Server] Failed to listen (errno %d)\n", errno);
				close(server_fd);
				server_fd = -1;
				k_sleep(K_MSEC(1000));
				continue;
			}

			printk("[HTTP Server] Listening on port %d for web clients (max %d SSE)...\n",
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

		/* Poll for incoming connections or client disconnects with 200ms timeout */
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
								sse_clients[slot].fd = client_fd;
								sse_clients[slot].last_sent_id = 0;
								printk("[HTTP] SSE client #%d connected on fd %d\n",
								       slot + 1, client_fd);
							} else {
								close(client_fd);
							}
						} else {
							/* All SSE client slots are full */
							const char *busy_resp =
								"HTTP/1.1 503 Service Unavailable\r\n"
								"Content-Type: text/plain\r\n"
								"Connection: close\r\n\r\n"
								"Max SSE clients reached\n";
							send_all(client_fd, busy_resp, strlen(busy_resp));
							close(client_fd);
							printk("[HTTP] Rejected SSE request: max %d clients reached\n",
							       MAX_SSE_CLIENTS);
						}
					}
					/* Match GET / or GET /index.html (HTML Dashboard) */
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
						printk("[HTTP] Served complete HTML dashboard (%zu bytes)\n",
						       sizeof(HTTP_HTML_PAGE) - 1);
					}
					/* Other requests (e.g. /favicon.ico) -> 404 */
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

		/* 2. Check if any SSE client disconnected */
		if (poll_ret > 0) {
			for (int k = 1; k < num_fds; k++) {
				int c_idx = pfd_to_client[k - 1];
				int cfd = sse_clients[c_idx].fd;
				if (cfd < 0) {
					continue;
				}

				if (pfd[k].revents & (ZSOCK_POLLERR | ZSOCK_POLLHUP)) {
					printk("[HTTP] SSE client #%d (fd %d) disconnected (hangup/error)\n",
					       c_idx + 1, cfd);
					close(cfd);
					sse_clients[c_idx].fd = -1;
				} else if (pfd[k].revents & ZSOCK_POLLIN) {
					/* Peer sent FIN or unexpected data; read to check EOF */
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

		/* 3. Push new telemetry frame to all active SSE clients */
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
			int pending_clients = 0;

			for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
				if (sse_clients[i].fd >= 0) {
					active_count++;
					if (sse_clients[i].last_sent_id != cur.sample_id) {
						pending_clients++;
					}
				}
			}

			if (pending_clients > 0) {
				int32_t temp_int  = cur.temp_mc / 1000;
				int32_t temp_frac = abs(cur.temp_mc % 1000) / 100;
				uint32_t volt_int  = cur.bus_mv / 1000;
				uint32_t volt_frac = (cur.bus_mv % 1000) / 10;

				char sse_data[192];
				int sse_len = snprintf(sse_data, sizeof(sse_data),
				                       "data: {\"id\":%u,\"rpm\":%u,\"temp\":%d.%d,\"volt\":%u.%02u,\"curr\":%u,\"uptime\":%lld,\"clients\":%d}\n\n",
				                       cur.sample_id,
				                       cur.motor_rpm,
				                       temp_int, temp_frac,
				                       volt_int, volt_frac,
				                       cur.current_ma,
				                       cur.uptime_ms,
				                       active_count);

				int pushed = 0;
				for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
					if (sse_clients[i].fd >= 0 &&
					    sse_clients[i].last_sent_id != cur.sample_id) {
						if (send_all(sse_clients[i].fd, sse_data, sse_len) < 0) {
							printk("[HTTP] SSE client #%d (fd %d) disconnected on write\n",
							       i + 1, sse_clients[i].fd);
							close(sse_clients[i].fd);
							sse_clients[i].fd = -1;
						} else {
							sse_clients[i].last_sent_id = cur.sample_id;
							pushed++;
						}
					}
				}

				if (pushed > 0) {
					printk("[HTTP] Broadcast SSE telemetry packet #%u to %d client(s)\n",
					       cur.sample_id, pushed);
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

K_THREAD_DEFINE(thread_logger, THREAD_STACK_SIZE, telemetry_logger_thread_fn,
                NULL, NULL, NULL, PRIORITY_LOW, 0, 0);

K_THREAD_DEFINE(thread_wifi_http, WIFI_STACK_SIZE, wifi_http_thread_fn,
                NULL, NULL, NULL, PRIORITY_WIFI, 0, 0);

/* ========================================================================= */
/* Main Application Entry Point                                              */
/* ========================================================================= */
int main(void)
{
	const struct device *const timer_dev = DEVICE_DT_GET(TIMER_NODE);
	int err;

	printk("=================================================================\n");
	printk("Waveshare ESP32-C5-Zero: Multi-Threaded Wi-Fi Web Dashboard\n");
	printk("Architecture: RISC-V 32-bit (ESP32-C5)\n");
	printk("HW Timer: %s (%s)\n", timer_dev->name, DT_NODE_PATH(TIMER_NODE));
	printk("=================================================================\n");

	if (!device_is_ready(timer_dev)) {
		printk("FATAL: Counter device %s is not ready!\n", timer_dev->name);
		return -ENODEV;
	}

	uint32_t freq = counter_get_frequency(timer_dev);
	printk("Timer frequency: %u Hz\n", freq);

	/* Configure periodic top value (1.0 second auto-reload) */
	struct counter_top_cfg top_cfg = {
		.ticks = counter_us_to_ticks(timer_dev, TIMER_INTERVAL_US),
		.callback = timer_top_subroutine,
		.user_data = NULL,
		.flags = 0,
	};

	printk("Starting hardware timer: period = 1s (%llu ticks)...\n",
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

	printk("Hardware timer running. Pipeline & Wi-Fi Server active.\n\n");

	/* Main thread can sleep or perform non-critical supervisory tasks */
	while (1) {
		k_sleep(K_SECONDS(30));
		printk("[Supervisor Heartbeat] System healthy, 64-bit uptime: %lld ms\n",
		       k_uptime_get());
	}

	return 0;
}
