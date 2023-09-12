/**
 * @file fusb302b.c
 * @author ottojo
 * @date 9/10/23
 * Description here TODO
 */

#define DT_DRV_COMPAT fcs_fusb302b

#include "fusb302b_private.h"
#include <zephyr/drivers/usb_c/fusb302b.h>
#include <zephyr/drivers/usb_c/usbc_tcpc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(fusb302b, LOG_LEVEL_DBG);

static const uint8_t REG_DEVICE_ID = 0x01;
static const uint8_t REG_SWITCHES0 = 0x02;
static const uint8_t REG_SWITCHES1 = 0x03;
static const uint8_t REG_MEASURE = 0x04;
static const uint8_t REG_CONTROL0 = 0x06;
static const uint8_t REG_CONTROL1 = 0x07;
static const uint8_t REG_CONTROL3 = 0x09;
static const uint8_t REG_POWER = 0x0b;
static const uint8_t REG_RESET = 0x0c;
static const uint8_t REG_STATUS0 = 0x40;
static const uint8_t REG_STATUS1 = 0x41;
static const uint8_t REG_FIFO = 0x43;


bool fusb302b_verify(const struct device *dev) {
    const struct fusb302b_cfg *cfg = dev->config;
    uint8_t device_id;
    int res = i2c_reg_read_byte_dt(&cfg->i2c, REG_DEVICE_ID, &device_id);
    if (res != 0) {
        return false;
    }
    const uint8_t version_id = device_id >> 4;
    const uint8_t product_id = (device_id >> 2) & 0b11;
    const uint8_t revision_id = device_id & 0b11;

    const char *product_str = 0;
    switch (product_id) {
        case 0b00:
            product_str = "FUSB302B(MP|VMP|UC)X";
            break;
        case 0b01:
            product_str = "FUSB302B01MPX";
            break;
        case 0b10:
            product_str = "FUSB302B10MPX";
            break;
        case 0b11:
            product_str = "FUSB302B11MPX";
            break;
        default: __ASSERT_NO_MSG(false);
            return -EIO;
    }

    const char *version_str = 0;
    switch (version_id) {
        case 0b1000:
            version_str = "A";
            break;
        case 0b1001:
            version_str = "B";
            break;
        case 0b1010:
            version_str = "C";
            break;
        default:
            return -ENODEV;
    }

    const char *revision_str = 0;
    switch (revision_id) {
        case 0b00:
            revision_str = "revA";
            break;
        case 0b01:
            revision_str = "revB";
            break;
        case 0b10:
            revision_str = "revC";
            break;
        case 0b11:
            revision_str = "revD";
            break;
        default: __ASSERT_NO_MSG(false);
            return -EIO;
    }
    LOG_INF("%s %s_%s detected\n", product_str, version_str, revision_str);

    return true;
}

static int vbus_level_to_mv(uint8_t level) {
    return (level + 1) * 420;
}

static bool vbus_above(const struct fusb302b_cfg *cfg, uint8_t level) {
    // Set MEAS_VBUS to 1, to measure VBUS with the MDAC/comparator
    int res = i2c_reg_write_byte_dt(&cfg->i2c, REG_MEASURE, 0b01000000 | (level & 0b111111));
    if (res != 0) { LOG_ERR("Error setting DAC to measure VBUS: %d", res); }
    uint8_t status0 = 0;
    res = i2c_reg_read_byte_dt(&cfg->i2c, REG_STATUS0, &status0);
    if (res != 0) { LOG_ERR("Error getting comparison to measure VBUS: %d", res); }
    bool above = (status0 & 0b00100000) != 0;
    LOG_DBG("VBUS %c%d mV", above ? '>' : '<', vbus_level_to_mv(level));
    return above;
}


int fusb302_measure_vbus(const struct device *dev, int *meas) {
    const struct fusb302b_cfg *cfg = dev->config;

    // Set MEAS_CC bits to 0
    int res = i2c_reg_write_byte_dt(&cfg->i2c, REG_SWITCHES0, 0x00000011);
    if (res != 0) { return -EIO; }


    uint8_t lower_bound = 0;
    uint8_t upper_bound = 0b111111;
    if (!vbus_above(cfg, lower_bound)) {
        return vbus_level_to_mv(lower_bound);
    }
    if (vbus_above(cfg, upper_bound)) {
        return vbus_level_to_mv(upper_bound);
    }

    // Binary search VBUS voltage
    while ((upper_bound - lower_bound) > 1) {
        uint8_t middle = (lower_bound + upper_bound) / 2;
        if (vbus_above(cfg, middle)) {
            lower_bound = middle;
        } else {
            upper_bound = middle;
        }
    }

    *meas = vbus_level_to_mv(upper_bound);
    return 0;
}

int fusb302_reset(const struct device *dev) {
    const struct fusb302b_cfg *cfg = dev->config;
    // SW_RES: "Reset the FUSB302B including the I2C registers to their default values"
    return i2c_reg_write_byte_dt(&cfg->i2c, REG_RESET, 0b00000001);
}

int fusb302_setup(const struct device *dev) {
    const struct fusb302b_cfg *cfg = dev->config;
    // Detect which CC line is in use

    // Connect ADC to CC1 (set MEAS_CC1)
    int res = i2c_reg_write_byte_dt(&cfg->i2c, REG_SWITCHES0, 0b00000111);
    if (res != 0) { return -EIO; }
    // Read voltage level
    uint8_t status0;
    res = i2c_reg_read_byte_dt(&cfg->i2c, REG_STATUS0, &status0);
    if (res != 0) { return -EIO; }
    uint8_t voltage_level_1 = status0 & 0b11;
    // Connect ADC to CC2
    res = i2c_reg_write_byte_dt(&cfg->i2c, REG_SWITCHES0, 0b00001011);
    if (res != 0) { return -EIO; }
    // Read voltage level
    res = i2c_reg_read_byte_dt(&cfg->i2c, REG_STATUS0, &status0);
    if (res != 0) { return -EIO; }
    uint8_t voltage_level_2 = status0 & 0b11;

    uint8_t used_cc_line;
    if (voltage_level_1 > voltage_level_2) {
        used_cc_line = 1;
    } else if (voltage_level_2 > voltage_level_1) {
        used_cc_line = 2;
    } else if (voltage_level_1 == 0 && voltage_level_2 == 0) {
        // No CC connected?
        LOG_WRN("No CC pins connected!\n");
        return -ENODEV;
    } else {
        LOG_WRN("Invalid CC voltage levels: BC_LVL 1: %d, BC_LVL 2: %d\n", voltage_level_1, voltage_level_2);
        return -ENODEV;
    }
    LOG_INF("CC pin in use: %d\n", used_cc_line);

    // Enable transmit driver for proper CC line, enable AUTO_CRC
    uint8_t cc_select = (used_cc_line == 1) ? 0b01 : 0b10;
    res = i2c_reg_write_byte_dt(&cfg->i2c, REG_SWITCHES1, 0b00100100 | cc_select);
    if (res != 0) { return -EIO; }
    // Connect measure block to proper CC line
    res = i2c_reg_write_byte_dt(&cfg->i2c, REG_SWITCHES0, 0b00000011 | (cc_select << 2));
    if (res != 0) { return -EIO; }

    // Flush TX buffer (TX_FLUSH)
    res = i2c_reg_write_byte_dt(&cfg->i2c, REG_CONTROL0, 0b01000000);
    if (res != 0) { return -EIO; }
    // Flush RX buffer
    res = i2c_reg_write_byte_dt(&cfg->i2c, REG_CONTROL1, 0b00000100);
    if (res != 0) { return -EIO; }
    // Reset PD logic
    res = i2c_reg_write_byte_dt(&cfg->i2c, REG_RESET, 0b00000010);
    if (res != 0) { return -EIO; }

    return 0;
}

int fusb302b_read_messages(const struct device *dev) {
    const struct fusb302b_cfg *cfg = dev->config;

    uint8_t status1;
    int res = i2c_reg_read_byte_dt(&cfg->i2c, REG_STATUS1, &status1);
    if (res != 0) { return -EIO; }
    uint8_t rx_empty = (status1 >> 5) & 0b1;

    if (rx_empty) {
        LOG_INF("RX buffer empty\n");
        return 0;
    }

    LOG_INF("RX buffer contains something, reading...\n");
    uint8_t rx_buffer[80];
    res = i2c_burst_read_dt(&cfg->i2c, REG_FIFO, rx_buffer, sizeof(rx_buffer));
    if (res != 0) { return -EIO; }

    return -ENODEV;
}

int fusb302b_init(const struct device *dev) {
    const struct fusb302b_cfg *cfg = dev->config;

    if (!i2c_is_ready_dt(&cfg->i2c)) {
        return -ENODEV;
    }

    if (fusb302_reset(dev) != 0) {
        return -EIO;
    }

    if (!fusb302b_verify(dev)) {
        return -EIO;
    }

    // Power up all parts of device
    int res = i2c_reg_write_byte_dt(&cfg->i2c, REG_POWER, 0b00001111);
    if (res != 0) {
        return -EIO;
    }

    // Unmask interrupts
    res = i2c_reg_write_byte_dt(&cfg->i2c, REG_CONTROL0, 0b00000000);
    if (res != 0) {
        return -EIO;
    }

    // Enable packet retries
    res = i2c_reg_write_byte_dt(&cfg->i2c, REG_CONTROL3, 0b00000111);
    if (res != 0) {
        return -EIO;
    }

    return 0;
}

// required == assumed by assertion, optional == results in ENOSYS
static const struct tcpc_driver_api fusb302b_tcpc_driver_api = {
        .init = NULL, // TODO Required
        .get_cc = NULL, // Optional
        .select_rp_value=NULL, // Optional
        .get_rp_value = NULL,// Optional
        .set_cc = NULL, // TODO Required
        .set_vconn_discharge_cb = NULL, // TODO Required
        .set_vconn_cb = NULL,// TODO Required
        .vconn_discharge = NULL,// Optional
        .set_vconn = NULL, // Optional
        .set_roles = NULL, // Optional
        .receive_data = NULL, // Optional
        .is_rx_pending_msg = NULL, // Optional
        .set_rx_enable = NULL, // Optional
        .set_cc_polarity = NULL, // TODO Required
        .transmit_data = NULL, // Optional
        .dump_std_reg = NULL, // Optional
        .alert_handler_cb=NULL, // Unused?
        .get_status_register=NULL, // Optional
        .clear_status_register=NULL, // Optional
        .mask_status_register=NULL, // Optional
        .set_debug_accessory=NULL, // Optional
        .set_debug_detach=NULL, // Optional
        .set_drp_toggle=NULL, // Optional
        .get_snk_ctrl=NULL, // Optional
        .get_src_ctrl=NULL, // Optional
        .get_chip_info=NULL, // Optional
        .set_low_power_mode=NULL, // Optional
        .sop_prime_enable=NULL, // Optional
        .set_bist_test_mode=NULL, // Optional
        .set_alert_handler_cb=NULL, // Required
};

#define FUSB302B_DEFINE(inst) \
    static struct fusb302b_data fusb302b_data_##inst; \
     \
    static const struct fusb302b_cfg fusb302b_config_##inst = { \
        .i2c = I2C_DT_SPEC_INST_GET(inst) \
    }; \
    DEVICE_DT_INST_DEFINE(inst, \
        fusb302b_init, \
        /* pm_device = */ NULL, \
        &fusb302b_data_##inst, \
        &fusb302b_config_##inst, \
        /* level = */ APPLICATION, \
        CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
        NULL /*&fusb302b_tcpc_driver_api*/  \
    );

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) > 0, "No compatible FUSB302B instance found");

DT_INST_FOREACH_STATUS_OKAY(FUSB302B_DEFINE)
