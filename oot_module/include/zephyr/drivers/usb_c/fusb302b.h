/**
 * @file fusb302b.h
 * @author ottojo
 * @date 9/10/23
 * Description here TODO
 */

#ifndef ZEPHYR_SANDBOX_2_FUSB302B_H
#define ZEPHYR_SANDBOX_2_FUSB302B_H

#include <stdbool.h>
#include <zephyr/device.h>


int fusb302_reset(const struct device *dev);

bool fusb302b_verify(const struct device *device);

int fusb302_setup(const struct device *dev);

int fusb302_measure_vbus(const struct device *dev, int *meas);

#endif //ZEPHYR_SANDBOX_2_FUSB302B_H
