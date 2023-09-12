#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/usb_c/fusb302b.h>
#include <zephyr/drivers/usb_c/usbc_vbus.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);


/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

const struct device *const fusb_dev = DEVICE_DT_GET(DT_NODELABEL(fusb));
const struct device *const vbus_dev = DEVICE_DT_GET(DT_NODELABEL(fvbus));


int main() {
    int ret;

    if (!gpio_is_ready_dt(&led)) {
        return 1;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 1;
    }

    int i = 0;
    while (true) {
        k_sleep(K_MSEC(5000));

        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            return 0;
        }

        if (device_is_ready(fusb_dev)) {
            LOG_INF("FUSB device ready\n");
            if (fusb302b_verify(fusb_dev)) {
                LOG_INF("FUSB device verified\n");
                if (fusb302_setup(fusb_dev) == 0) {
                    LOG_INF("FUSB setup successful\n");
                } else {
                    LOG_ERR("FUSB setup unsuccessful\n");
                }

                int meas = 0;
                if (usbc_vbus_measure(vbus_dev, &meas) == 0) {
                    LOG_INF("VBUS Measurement successful: %d mV", meas);
                } else {
                    LOG_ERR("VBUS Measurement unsuccessful");
                }
            } else {
                LOG_ERR("FUSB device not verified\n");
            }
        } else {
            LOG_ERR("FUSB device not ready\n");
        }
        i++;
    }
}
