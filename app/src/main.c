#include <stdbool.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/usb_c/fusb302b.h>


/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


const struct device *const fusb_dev = DEVICE_DT_GET(DT_NODELABEL(fusb));


int main() {
    int ret;

    if (!gpio_is_ready_dt(&led)) {
        return 1;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 1;
    }

    if (usb_enable(NULL)) {
        return 1;
    }


    k_sleep(K_SECONDS(5));

    int res = fusb302_setup(fusb_dev);
    if (res == 0) {
        printk("USB setup successful!\n");
    } else {
        printk("USB setup not successful!\n");
    }



    int i = 0;
    while (true) {
        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            return 0;
        }

        if (device_is_ready(fusb_dev)) {
            printk("FUSB device ready\n");
            if (fusb302b_verify(fusb_dev)) {
                printk("FUSB device verified\n");
            } else {
                printk("FUSB device not verified\n");
            }
        } else {
            printk("FUSB device not ready\n");
        }

        k_sleep(K_MSEC(1000));
        i++;
    }
}
