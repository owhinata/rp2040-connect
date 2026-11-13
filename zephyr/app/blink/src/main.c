#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define BLINK_INTERVAL_MS 500

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main(void)
{
	if (!gpio_is_ready_dt(&led)) {
		return 1;
	}

	if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE) < 0) {
		return 1;
	}

	while (1) {
		if (gpio_pin_toggle_dt(&led) < 0) {
			return 1;
		}
		k_msleep(BLINK_INTERVAL_MS);
	}
}
