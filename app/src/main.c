#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/usb_c/fusb302b.h>
#include <zephyr/drivers/usb_c/usbc_vbus.h>
#include <zephyr/usb_c/usbc.h>
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

const struct device *const port1 = DEVICE_DT_GET(DT_NODELABEL(port1));


static void display_pdo(const int idx,
                        const uint32_t pdo_value) {
    union pd_fixed_supply_pdo_source pdo;

    /* Default to fixed supply pdo source until type is detected */
    pdo.raw_value = pdo_value;

    LOG_INF("PDO %d:", idx);
    switch (pdo.type) {
        case PDO_FIXED: {
            LOG_INF("\tType:              FIXED");
            LOG_INF("\tCurrent:           %d",
                    PD_CONVERT_FIXED_PDO_CURRENT_TO_MA(pdo.max_current));
            LOG_INF("\tVoltage:           %d",
                    PD_CONVERT_FIXED_PDO_VOLTAGE_TO_MV(pdo.voltage));
            LOG_INF("\tPeak Current:      %d", pdo.peak_current);
            LOG_INF("\tUchunked Support:  %d",
                    pdo.unchunked_ext_msg_supported);
            LOG_INF("\tDual Role Data:    %d",
                    pdo.dual_role_data);
            LOG_INF("\tUSB Comms:         %d",
                    pdo.usb_comms_capable);
            LOG_INF("\tUnconstrained Pwr: %d",
                    pdo.unconstrained_power);
            LOG_INF("\tUSB Suspend:       %d",
                    pdo.usb_suspend_supported);
            LOG_INF("\tDual Role Power:   %d",
                    pdo.dual_role_power);
        }
            break;
        case PDO_BATTERY: {
            union pd_battery_supply_pdo_source batt_pdo;

            batt_pdo.raw_value = pdo_value;
            LOG_INF("\tType:              BATTERY");
            LOG_INF("\tMin Voltage: %d",
                    PD_CONVERT_BATTERY_PDO_VOLTAGE_TO_MV(batt_pdo.min_voltage));
            LOG_INF("\tMax Voltage: %d",
                    PD_CONVERT_BATTERY_PDO_VOLTAGE_TO_MV(batt_pdo.max_voltage));
            LOG_INF("\tMax Power:   %d",
                    PD_CONVERT_BATTERY_PDO_POWER_TO_MW(batt_pdo.max_power));
        }
            break;
        case PDO_VARIABLE: {
            union pd_variable_supply_pdo_source var_pdo;

            var_pdo.raw_value = pdo_value;
            LOG_INF("\tType:        VARIABLE");
            LOG_INF("\tMin Voltage: %d",
                    PD_CONVERT_VARIABLE_PDO_VOLTAGE_TO_MV(var_pdo.min_voltage));
            LOG_INF("\tMax Voltage: %d",
                    PD_CONVERT_VARIABLE_PDO_VOLTAGE_TO_MV(var_pdo.max_voltage));
            LOG_INF("\tMax Current: %d",
                    PD_CONVERT_VARIABLE_PDO_CURRENT_TO_MA(var_pdo.max_current));
        }
            break;
        case PDO_AUGMENTED: {
            union pd_augmented_supply_pdo_source aug_pdo;

            aug_pdo.raw_value = pdo_value;
            LOG_INF("\tType:              AUGMENTED");
            LOG_INF("\tMin Voltage:       %d",
                    PD_CONVERT_AUGMENTED_PDO_VOLTAGE_TO_MV(aug_pdo.min_voltage));
            LOG_INF("\tMax Voltage:       %d",
                    PD_CONVERT_AUGMENTED_PDO_VOLTAGE_TO_MV(aug_pdo.max_voltage));
            LOG_INF("\tMax Current:       %d",
                    PD_CONVERT_AUGMENTED_PDO_CURRENT_TO_MA(aug_pdo.max_current));
            LOG_INF("\tPPS Power Limited: %d", aug_pdo.pps_power_limited);
        }
            break;
    }
}

uint32_t policy_cb_get_rdo(const struct device *dev) {
// * @brief Get a Request Data Object from the DPM
    // Get selected source cap from Device Policy Manager

    // Build a Request Data Object

    union pd_rdo rdo;
    /* Maximum operating current 100mA (GIVEBACK = 0) */
    rdo.fixed.min_or_max_operating_current = PD_CONVERT_MA_TO_FIXED_PDO_CURRENT(100);
    /* Operating current 100mA */
    rdo.fixed.operating_current = PD_CONVERT_MA_TO_FIXED_PDO_CURRENT(100);
    /* Unchunked Extended Messages Not Supported */
    rdo.fixed.unchunked_ext_msg_supported = 0;
    /* No USB Suspend */
    rdo.fixed.no_usb_suspend = 1;
    /* Not USB Communications Capable */
    rdo.fixed.usb_comm_capable = 0;
    /* No capability mismatch */
    rdo.fixed.cap_mismatch = 0;
    /* Don't giveback */
    rdo.fixed.giveback = 0;
    /* Object position 1 (5V PDO) */
    // TODO: Find out which source PDO lists the voltage we want. 1 is special for 5v or something
    rdo.fixed.object_pos = 1;

    return rdo.raw_value;

}

void handle_src_caps(const struct device *dev, const uint32_t *pdos,
                     const int num_pdos) {
    for (int i = 0; i < num_pdos; i++) {
        display_pdo(i, pdos[i]);
    }
}


int main() {
    int ret;

    if (!gpio_is_ready_dt(&led)) {
        return 1;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 1;
    }

    usbc_set_policy_cb_set_src_cap(port1, handle_src_caps);

    //usbc_set_policy_cb_get_rdo(port1, policy_cb_get_rdo);

    LOG_INF("Starting USB-C");
    usbc_start(port1);

    while (true) {
        gpio_pin_toggle_dt(&led);
        k_sleep(K_MSEC(1000));
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
