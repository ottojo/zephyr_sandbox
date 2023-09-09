#include <stdbool.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

const struct device *const console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));


int main() {
    int ret;

    if (!gpio_is_ready_dt(&led)) {
        return 1;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 1;
    }


    uint32_t dtr = 0;

    if (usb_enable(NULL)) {
        return 1;
    }

    //while (!dtr) {
    //    uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
    //    k_sleep(K_MSEC(100));
    //}

    int i = 0;
    while (true) {
        printk("Hello World %d\n", i);
        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            return 0;
        }
        k_sleep(K_MSEC(1000));
        i++;
    }
}
