/**
 * @file fusb302b.c
 * @author ottojo
 * @date 9/10/23
 * Description here TODO
 */

#define DT_DRV_COMPAT fcs_fusb302b

#include "fusb302b_private.h"
#include <zephyr/drivers/usb_c/fusb302b.h>

static const uint8_t REG_DEVICE_ID = 0x01;
static const uint8_t REG_SWITCHES0 = 0x02;
static const uint8_t REG_SWITCHES1 = 0x03;
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

    switch (product_id) {
        case 0b00:
            printk("FUSB302B(MP|VMP|UC)X ");
            break;
        case 0b01:
            printk("FUSB302B01MPX ");
            break;
        case 0b10:
            printk("FUSB302B10MPX ");
            break;
        case 0b11:
            printk("FUSB302B11MPX ");
            break;
        default:
            __builtin_unreachable();
    }

    switch (version_id) {
        case 0b1000:
            printk("A_");
            break;
        case 0b1001:
            printk("B_");
            break;
        case 0b1010:
            printk("C_");
            break;
        default:
            return -ENODEV;
    }

    switch (revision_id) {
        case 0b00:
            printk("revA");
            break;
        case 0b01:
            printk("revB");
            break;
        case 0b10:
            printk("revC");
            break;
        case 0b11:
            printk("revD");
            break;
        default:
            __builtin_unreachable();
    }
    printk(" detected\n");

    return true;
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
        printk("No CC pins connected!\n");
        return -ENODEV;
    } else {
        printk("Invalid CC voltage levels: BC_LVL 1: %d, BC_LVL 2: %d\n", voltage_level_1, voltage_level_2);
        return -ENODEV;
    }
    printk("CC pin in use: %d\n", used_cc_line);

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
        printk("RX buffer empty\n");
        return 0;
    }

    printk("RX buffer contains something, reading...\n");
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
        /* api = */ NULL \
    );

DT_INST_FOREACH_STATUS_OKAY(FUSB302B_DEFINE)
