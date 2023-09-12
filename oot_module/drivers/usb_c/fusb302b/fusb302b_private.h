/**
 * @file fusb302b_private.h
 * @author ottojo
 * @date 9/10/23
 * Description here TODO
 */

#ifndef ZEPHYR_SANDBOX_2_FUSB302B_PRIVATE_H
#define ZEPHYR_SANDBOX_2_FUSB302B_PRIVATE_H

#include <zephyr/drivers/i2c.h>

struct fusb302b_data {
};

struct fusb302b_cfg {
    struct i2c_dt_spec i2c;
};

int fusb302b_init(const struct device *dev);


#endif //ZEPHYR_SANDBOX_2_FUSB302B_PRIVATE_H
