/**
 * @file fusb302b_private.h
 * @author ottojo
 * @date 9/10/23
 * Description here TODO
 */

#ifndef ZEPHYR_SANDBOX_2_FUSB302B_PRIVATE_H
#define ZEPHYR_SANDBOX_2_FUSB302B_PRIVATE_H

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/usb_c/usbc_tcpc.h>
#include <zephyr/drivers/gpio.h>

#define FUSB302_RX_BUFFER_SIZE 80


struct alert_info {
    void *data;
    tcpc_alert_handler_cb_t handler;
};

struct fusb302b_data {
    struct alert_info alert_info;
};

struct fusb302b_cfg {
    struct i2c_dt_spec i2c;
};

int fusb302b_init(const struct device *dev);


#endif //ZEPHYR_SANDBOX_2_FUSB302B_PRIVATE_H
