# Target Architecture: BLDC Motor Controller & Telemetry System

This document outlines the target production architecture for the **Waveshare ESP32-S3-GEEK**, transitioning from the prototype simulation on the ESP32-C5-Zero to full hardware implementation.

---

## 1. System Overview

The system is a closed-loop **Brushless DC (BLDC) Motor Controller & Data Logger** with:
- Multi-stage speed profiles with timed stages.
- Real-time RPM measurement via motor Frequency Generator (FG) feedback.
- Thermal and electrical monitoring (DS18B20 temperature, INA219 current/voltage).
- Persistent logging to an onboard MicroSD card.
- Live telemetry displayed on the 1.14" ST7789 LCD and served over a Wi-Fi HTTP interface.

---

## 2. Hardware Peripheral Mapping (ESP32-S3-GEEK)

| Peripheral / Signal | Hardware Source | Zephyr Subsystem / Driver | Purpose |
|---|---|---|---|
| **Motor FG Feedback** | GPIO Pin (Interrupt/ETM) | `gpio_pin_interrupt_configure_dt` + Counter | Measure pulse frequency to calculate exact motor RPM |
| **Motor PWM Control** | GPIO Pin (LEDC / MCPWM) | `zephyr/drivers/pwm.h` (`pwm_esp32`) | Set motor throttle across test stages |
| **Motor Temperature** | DS18B20 (1-Wire) | `zephyr/drivers/w1.h` or bit-banged GPIO | Monitor motor operating temperature |
| **Power Monitoring** | INA219 (I2C0) | `zephyr/drivers/sensor/ina219.h` | Bus voltage (V), shunt current (mA), and power (mW) |
| **Data Logging** | Onboard MicroSD Slot | `sdhc0` (`zephyr,sdmmc-disk`) + `FATFS` | Write persistent CSV telemetry files |
| **Local Display** | Onboard 1.14" ST7789 LCD | `st7789p3` (`zephyr/drivers/display.h`) | Real-time status, RPM, stage timer, and alarms |
| **Web Telemetry** | ESP32-S3 Wi-Fi 802.11 b/g/n | `zephyr/net/socket.h` + HTTP Server | Provide browser-based live JSON and HTML dashboard |

---

## 3. Multi-Threaded RTOS Architecture

```text
 ┌──────────────────────────────────────────────────────────────────────────┐
 │                      Hardware Timer Interrupt (1 kHz / 100 Hz)          │
 └────────────────────────────────────┬─────────────────────────────────────┘
                                      │ Signals Semaphore
                                      ▼
 ┌──────────────────────────────────────────────────────────────────────────┐
 │ [Priority 2 - Real-Time] Thread 1: Motor FG & Speed Engine              │
 │ - Counts FG pulses over fixed time windows                               │
 │ - Calculates RPM: (Pulses * 60) / (Poles * Window_Sec)                   │
 │ - Pushes sample to motor_speed_msgq                                      │
 └────────────────────────────────────┬─────────────────────────────────────┘
                                      │ motor_speed_msgq
                                      ▼
 ┌──────────────────────────────────────────────────────────────────────────┐
 │ [Priority 4 - Control] Thread 2: Stage Controller & Sensor Aggregator    │
 │ - Manages timed motor stages (e.g. Stage 1: 50% for 30s, Stage 2: 80%...)│
 │ - Updates Motor PWM duty cycle                                           │
 │ - Reads I2C INA219 (V, I) & 1-Wire DS18B20 (Temp)                       │
 │ - Pushes telemetry_packet_t to telemetry_msgq                            │
 └────────────────────────────────────┬─────────────────────────────────────┘
                                      │ telemetry_msgq
                  ┌───────────────────┴───────────────────┐
                  ▼                                       ▼
 ┌─────────────────────────────────┐   ┌──────────────────────────────────┐
 │ [Priority 6] Thread 3: Display  │   │ [Priority 8] Thread 4: Background│
 │ - Renders status to ST7789 LCD  │   │   SD Logger & HTTP Server        │
 │ - Shows Stage, Remaining Time,  │   │ - Batches writes to MicroSD card │
 │   RPM, Temp, and Current        │   │ - Serves HTTP polling requests   │
 └─────────────────────────────────┘   └──────────────────────────────────┘
```

### Thread Priority Allocation

| Thread | Priority | Description | Execution Budget |
|---|---|---|---|
| **Thread 1: Speed Engine** | `2` (High) | FG pulse calculation, emergency overspeed detection | Fast (~100 µs), non-blocking |
| **Thread 2: Stage Controller** | `4` (Med-High) | PWM stage progression, I2C/1-Wire sensor aggregation | Periodic (~5–10 ms) |
| **Thread 3: LCD Dashboard** | `6` (Medium) | SPI display frame refresh (ST7789) | 10–20 Hz update rate |
| **Thread 4: Storage & Wi-Fi** | `8` (Low) | MicroSD block writes (FATFS) and HTTP client requests | Can yield safely to higher priorities |

---

## 4. Devicetree Blueprint (`ESP32-S3-GEEK/app/app.overlay`)

When migrating to the physical ESP32-S3-GEEK board, the overlay will configure:

```dts
/ {
	aliases {
		app-timer = &counter0;
		motor-pwm = &pwm0;
		motor-fg  = &gpio_fg;
	};

	chosen {
		zephyr,console = &usb_serial;
		zephyr,shell-uart = &usb_serial;
		zephyr,display = &st7789p3;
	};
};

/* Enable I2C0 for INA219 Power Monitor */
&i2c0 {
	status = "okay";
	ina219: ina219@40 {
		compatible = "ti,ina219";
		reg = <0x40>;
		status = "okay";
		brng = <1>;
		pg = <3>;
		sadc = <13>;
		badc = <13>;
		shunt-resistor = <100000>; /* 0.1 Ohm */
		lsb-microvolt = <10>;
	};
};

/* Enable MicroSD Card Slot */
&sdhc {
	status = "okay";
	sdhc0: sdhc@0 {
		status = "okay";
		mmc {
			status = "okay";
		};
	};
};

/* Enable Hardware Counter for Timing Windows */
&timer0 {
	status = "okay";
	counter0: counter {
		status = "okay";
	};
};
```

---

## 5. Implementation Roadmap

1. **Phase 1: Multi-Thread Pipeline Simulation (Done on ESP32-C5-Zero)**
   - 3 prioritized threads communicating over `k_msgq`.
   - Hardware counter triggering speed engine.
   - Verified non-blocking message passing.

2. **Phase 2: ESP32-S3-GEEK Hardware Bring-Up (When board arrives)**
   - Configure GPIO interrupt for motor FG input.
   - Configure PWM driver (`pwm_set_dt`) for motor throttle control.
   - Implement stage countdown timer.

3. **Phase 3: Sensor Integration**
   - Wire INA219 to I2C0 (`SDA = GPIO 6`, `SCL = GPIO 7`).
   - Wire DS18B20 to designated GPIO.
   - Replace simulated data in Thread 2 with real sensor drivers.

4. **Phase 4: MicroSD & Wi-Fi Server**
   - Initialize FATFS on `sdhc0`.
   - Write CSV headers and log entries in Thread 4.
   - Initialize Wi-Fi station mode and HTTP listener on port 80.

5. **Phase 5: ST7789 LCD Dashboard**
   - Use Zephyr Display API or LVGL to draw a real-time motor dashboard.

