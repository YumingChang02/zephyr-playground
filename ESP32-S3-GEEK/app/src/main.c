#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void)
{
	printk("Hello from Waveshare ESP32-S3-GEEK running Zephyr!\n");

	while (1) {
		k_msleep(1000);
		printk("Heartbeat...\n");
	}

	return 0;
}

