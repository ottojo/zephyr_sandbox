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

LOG_MODULE_REGISTER(fusb302b, LOG_LEVEL_WRN);

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

static const uint8_t TX_TOKEN_TXON = 0xA1;
static const uint8_t TX_TOKEN_SOP1 = 0x12;
static const uint8_t TX_TOKEN_SOP2 = 0x13;
static const uint8_t TX_TOKEN_PACKSYM = 0x80;
static const uint8_t TX_TOKEN_JAM_CRC = 0xFF;
static const uint8_t TX_TOKEN_EOP = 0x14;
static const uint8_t TX_TOKEN_TXOFF = 0xFE;

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
        default:
            __ASSERT_NO_MSG(false);
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
        default:
            __ASSERT_NO_MSG(false);
            return -EIO;
    }
    LOG_INF("%s %s_%s detected", product_str, version_str, revision_str);

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

int fusb302_check_vbus_level(const struct device *dev, enum tc_vbus_level level) {
    const struct fusb302b_cfg *cfg = dev->config;

    switch (level) {
        case TC_VBUS_SAFE0V:
            return !vbus_above(cfg, 0 /* 420mv */);
        case TC_VBUS_PRESENT:
            return vbus_above(cfg, 10 /* 4620mV */);
        case TC_VBUS_REMOVED:
            return !vbus_above(cfg, 7 /* 3360mV */);
        default:
            LOG_ERR("Invalid value for tc_vbus_level: %d", level);
            break;
    }
    return false;
}

int fusb302_measure_vbus(const struct device *dev, int *meas) {
    const struct fusb302b_cfg *cfg = dev->config;
    LOG_WRN("Measuring VBUS exactly, this may be slow");

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
    LOG_DBG("Measured VBUS at %dmV", *meas);
    return 0;
}

int fusb302_reset(const struct device *dev) {
    const struct fusb302b_cfg *cfg = dev->config;
    // SW_RES: "Reset the FUSB302B including the I2C registers to their default values"
    return i2c_reg_write_byte_dt(&cfg->i2c, REG_RESET, 0b00000001);
}

enum cc_res {
    CC_RES_1,
    CC_RES_2,
    CC_RES_BOTH,
    CC_RES_NONE
};

static const char *cc_res_to_str(enum cc_res res) {
    switch (res) {
        case CC_RES_1:
            return "CC_RES_1";
        case CC_RES_2:
            return "CC_RES_2";
        case CC_RES_BOTH:
            return "CC_RES_BOTH";
        case CC_RES_NONE:
            return "CC_RES_NONE";
    }
    return "<INVALID>";
}

static int get_cc_line(const struct fusb302b_cfg *cfg, enum cc_res *out) {
    // Connect ADC to CC1 (set MEAS_CC1)
    int res = i2c_reg_write_byte_dt(&cfg->i2c, REG_SWITCHES0, 0b00000111);
    if (res != 0) { return -EIO; }
    k_usleep(250);
    // Read voltage level BC_LVL
    uint8_t status0;
    res = i2c_reg_read_byte_dt(&cfg->i2c, REG_STATUS0, &status0);
    if (res != 0) { return -EIO; }
    uint8_t voltage_level_1 = status0 & 0b11;
    // Connect ADC to CC2
    res = i2c_reg_write_byte_dt(&cfg->i2c, REG_SWITCHES0, 0b00001011);
    if (res != 0) { return -EIO; }
    k_usleep(250);
    // Read voltage level BC_LVL
    res = i2c_reg_read_byte_dt(&cfg->i2c, REG_STATUS0, &status0);
    if (res != 0) { return -EIO; }
    uint8_t voltage_level_2 = status0 & 0b11;

    if (voltage_level_1 > voltage_level_2) {
        //LOG_INF("CC pin in use: %d", 1);
        *out = CC_RES_1;
    } else if (voltage_level_2 > voltage_level_1) {
        //LOG_INF("CC pin in use: %d", 2);
        *out = CC_RES_2;
    } else if (voltage_level_1 == 0 && voltage_level_2 == 0) {
        // No CC connected?
        //LOG_WRN("No CC pins connected!");
        *out = CC_RES_NONE;
    } else {
        LOG_WRN("Invalid CC voltage levels: BC_LVL 1: %d, BC_LVL 2: %d", voltage_level_1, voltage_level_2);
        *out = CC_RES_BOTH;
    }

    // TODO: BC_LVL 0b11 special? 3A?
    // 11: >1.23V why both tho?

    return 0;
}

int fusb302_setup(const struct device *dev) {
    LOG_INF("Running setup");
    const struct fusb302b_cfg *cfg = dev->config;

    LOG_INF("Flushing TX and RX buffers");
    // Flush TX buffer (TX_FLUSH)
    int res = i2c_reg_write_byte_dt(&cfg->i2c, REG_CONTROL0, 0b01000000);
    if (res != 0) { return -EIO; }
    // Flush RX buffer
    res = i2c_reg_write_byte_dt(&cfg->i2c, REG_CONTROL1, 0b00000100);
    if (res != 0) { return -EIO; }
    // Reset PD logic
    LOG_INF("Resetting PD logic");
    res = i2c_reg_write_byte_dt(&cfg->i2c, REG_RESET, 0b00000010);
    if (res != 0) { return -EIO; }

    LOG_INF("Setup complete");
    return 0;
}

int fusb302b_init(const struct device *dev) {
    LOG_INF("Init");
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

    int res = 0;

    // Disable Rd
    //LOG_INF("Disabling Rd");
    //res = i2c_reg_write_byte_dt(&cfg->i2c, REG_SWITCHES0, 0b00000000);
    //if (res != 0) { return -EIO; }

    // Power up all parts of device
    LOG_INF("Power up");
    res = i2c_reg_write_byte_dt(&cfg->i2c, REG_POWER, 0b00001111);
    if (res != 0) {
        return -EIO;
    }

    // Unmask interrupts
    LOG_INF("Enabling interrupts");
    // TODO: This selects non-default HOST_CUR (Rp) (off instead of usb default power) (but Rp is not enabled anyways...)
    res = i2c_reg_write_byte_dt(&cfg->i2c, REG_CONTROL0, 0b00000000);
    if (res != 0) { return -EIO; }

    // Enable packet retries
    LOG_INF("Enabling retries");
    res = i2c_reg_write_byte_dt(&cfg->i2c, REG_CONTROL3, 0b00000111);
    if (res != 0) {
        return -EIO;
    }

    LOG_INF("Init done");
    return 0;
}

static const char *cc_pull_to_str(enum tc_cc_pull cc_pull) {
    switch (cc_pull) {
        case TC_CC_RA:
            return "TC_CC_RA";
        case TC_CC_RP:
            return "TC_CC_RP";
        case TC_CC_RD:
            return "TC_CC_RD";
        case TC_CC_OPEN:
            return "TC_CC_OPEN";
        case TC_RA_RD:
            return "TC_RA_RD";
    }
    return "<invalid tc_cc_pull value>";
}

static int fusb302b_set_cc(const struct device *dev, enum tc_cc_pull cc_pull) {
    LOG_INF("Setting both CC to %s", cc_pull_to_str(cc_pull));

    const struct fusb302b_cfg *cfg = dev->config;

    switch (cc_pull) {
        case TC_CC_RA:
            LOG_ERR("Ra not supported.");
            return -ENOSYS;
        case TC_CC_RP: {
            LOG_ERR("Rp not supported.");
            return -ENOSYS;
        }
        case TC_CC_RD: {
            // Device pull down PDWN
            uint8_t switches0 = 0;
            int res = i2c_reg_read_byte_dt(&cfg->i2c, REG_SWITCHES0, &switches0);
            if (res != 0) { return -EIO; }
            switches0 |= 0b00000011; // set PDWN1,2
            switches0 &= 0b00111111; // unset PU_EN1,2
            res = i2c_reg_write_byte_dt(&cfg->i2c, REG_SWITCHES0, switches0);
            if (res != 0) { return -EIO; }
            break;
        }
        case TC_CC_OPEN: {
            LOG_WRN("IGNORING");
            return 0;
            uint8_t switches0 = 0;
            int res = i2c_reg_read_byte_dt(&cfg->i2c, REG_SWITCHES0, &switches0);
            if (res != 0) { return -EIO; }
            switches0 &= 0b00111100;
            res = i2c_reg_write_byte_dt(&cfg->i2c, REG_SWITCHES0, switches0);
            if (res != 0) { return -EIO; }
            break;
        }
        case TC_RA_RD:
            return -ENOSYS;
    }
    return 0;
}

static void fusb302b_set_vconn_discharge_cb(const struct device *dev, tcpc_vconn_discharge_cb_t cb) {
    struct fusb302b_data *data = dev->data;
    data->vconn_discharge_cb = cb;
}

static void fusb302b_set_vconn_cb(const struct device *dev, tcpc_vconn_control_cb_t vconn_cb) {
    struct fusb302b_data *data = dev->data;
    data->vconn_cb = vconn_cb;
}

static void bc_lvl_to_cc_state(uint8_t bc_lvl, enum tc_cc_voltage_state *cc) {
    switch (bc_lvl & 0b11) {
        case 0b00:
            *cc = TC_CC_VOLT_RA;
            break;
        case 0b01:
            *cc = TC_CC_VOLT_RD;
            break;
        case 0b10:
            break;
        case 0b11:
            break;
    }
}

static const char *cc_voltage_state_to_str(enum tc_cc_voltage_state cc_state) {
    switch (cc_state) {
        case TC_CC_VOLT_OPEN:
            return "TC_CC_VOLT_OPEN";
        case TC_CC_VOLT_RA:
            return "TC_CC_VOLT_RA";
        case TC_CC_VOLT_RD:
            return "TC_CC_VOLT_RD";
        case TC_CC_VOLT_RP_DEF:
            return "TC_CC_VOLT_RP_DEF";
        case TC_CC_VOLT_RP_1A5:
            return "TC_CC_VOLT_RP_1A5";
        case TC_CC_VOLT_RP_3A0:
            return "TC_CC_VOLT_RP_3A0";
    }
    return "<invalid tc_cc_voltage_state value>";
}

static int fusb302b_get_cc(const struct device *dev, enum tc_cc_voltage_state *cc1, enum tc_cc_voltage_state *cc2) {
    const struct fusb302b_cfg *cfg = dev->config;
    //LOG_INF("Determining CC state");
    // TODO: Determine voltage (and therefore RP type), see https://hackaday.com/2023/01/04/all-about-usb-c-resistors-and-emarkers/
    enum cc_res cc;
    int res = get_cc_line(cfg, &cc);
    if (res != 0) { return -EIO; }

    //LOG_INF("Determined CC state %s", cc_res_to_str(cc));
    switch (cc) {
        case CC_RES_1:
            *cc1 = TC_CC_VOLT_RP_DEF;
            *cc2 = TC_CC_VOLT_OPEN;
            break;
        case CC_RES_2:
            *cc1 = TC_CC_VOLT_OPEN;
            *cc2 = TC_CC_VOLT_RP_DEF;
            break;
        case CC_RES_BOTH:
            *cc1 = TC_CC_VOLT_RP_DEF;
            *cc2 = TC_CC_VOLT_RP_DEF;
            break;
        case CC_RES_NONE:
            *cc1 = TC_CC_VOLT_OPEN;
            *cc2 = TC_CC_VOLT_OPEN;
            break;
    }

    return 0;

    /*
     * The software determines if an Ra or Rd
termination is present based on the BC_LVL and COMP
interrupt and status bits.
     Additionally, for Rd terminations, the software can
further determine what charging current is allowed by the
Type-C host by reading the BC_LVL status bits. This is
summarized in Table 5
     */
    return 0;
}

/**
 * @brief Retrieves the Power Delivery message from the TCPC
 *
 * @param dev  Runtime device structure
 * @param buf  pointer where the pd_buf pointer is written
 *
 * @retval Greater or equal to 0 is the number of bytes received
 * @retval -EIO on failure
 * @retval -EFAULT on buf being NULL
 * @retval -ENOSYS if not implemented
 */
static int fusb302b_receive_data(const struct device *dev, struct pd_msg *buf) {
    if (buf == NULL) {
        LOG_ERR("Nullptr passed to fusb302b_receive_data");
        return -EFAULT;
    }

    const struct fusb302b_cfg *cfg = dev->config;

    uint8_t status1;
    int res = i2c_reg_read_byte_dt(&cfg->i2c, REG_STATUS1, &status1);
    if (res != 0) { return -EIO; }
    uint8_t rx_empty = (status1 >> 5) & 0b1;

    if (rx_empty) {
        return -EIO;
    }

    uint8_t rx_buffer[FUSB302_RX_BUFFER_SIZE];
    BUILD_ASSERT(sizeof(rx_buffer) <= sizeof(buf->data), "");
    res = i2c_burst_read_dt(&cfg->i2c, REG_FIFO, rx_buffer, sizeof(rx_buffer));
    if (res != 0) { return -EIO; }

    LOG_HEXDUMP_INF(rx_buffer, sizeof(rx_buffer), "Received data");

    // First byte determines package type
    uint8_t fusb_type = rx_buffer[0] >> 5;
    switch (fusb_type) {
        case 0b111:
            buf->type = PD_PACKET_SOP;
            break;
        case 0b110:
            buf->type = PD_PACKET_SOP_PRIME;
            break;
        case 0b101:
            buf->type = PD_PACKET_PRIME_PRIME;
            break;
        case 0b100:
            buf->type = PD_PACKET_DEBUG_PRIME;
            break;
        case 0b011:
            buf->type = PD_PACKET_DEBUG_PRIME_PRIME;
            break;
        default:
            return -EIO;
    }

    buf->header.raw_value = rx_buffer[1] | (rx_buffer[2] << 8);
    buf->len = PD_CONVERT_PD_HEADER_COUNT_TO_BYTES(buf->header.number_of_data_objects);
    LOG_INF("Received %d data objects", buf->header.number_of_data_objects);
    __ASSERT(buf->len + 3 <= sizeof(rx_buffer), "PD message size greater than RX buffer");
    memcpy(buf->data, rx_buffer + 1 /* package type */ + 2 /* header */, buf->len);

    return buf->len + 2 /* header */;
}

static bool fusb302b_is_rx_pending_msg(const struct device *dev, enum pd_packet_type *type) {
    LOG_WRN("Checking if rx pending msg");
    return false;
}

static int fusb302b_set_rx_enable(const struct device *dev, bool enable) {
    LOG_WRN("TODO: Setting RX enabled");
    return -ENOSYS;
}

int fusb302b_set_cc_polarity(const struct device *dev, enum tc_cc_polarity polarity) {
    const struct fusb302b_cfg *cfg = dev->config;
    // Enable transmit driver for proper CC line
    uint8_t cc_select = (polarity == TC_POLARITY_CC1) ? 0b01 : 0b10;
    LOG_INF("Enabling TX driver for CC %d and AUTO_CRC", cc_select);
    int res = i2c_reg_write_byte_dt(&cfg->i2c, REG_SWITCHES1, 0b00100100 | cc_select);
    if (res != 0) { return -EIO; }
    return 0;
}

int fusb302b_set_alert_handler_cb(const struct device *dev, tcpc_alert_handler_cb_t handler, void *alert_data) {
    struct fusb302b_data *data = dev->data;
    // TODO: Actually call that handler at some point
    data->alert_info.handler = handler;
    data->alert_info.data = alert_data;
    return 0;
}

int fusb302b_transmit_data(const struct device *dev, struct pd_msg *msg) {
    LOG_ERR("TRANSMITTING");
    // TODO verify size < FIFO size?
    // TODO: Flush TX buffer
    const struct fusb302b_cfg *cfg = dev->config;
    const struct fusb302b_data *data = dev->data;


    //__ASSERT_NO_MSG(msg->type == PD_PACKET_SOP);
    if (msg->type != PD_PACKET_SOP) {
        LOG_ERR("Packet type not supported");
        return -ENOSYS;
    }

    if (msg->len > (48 - 2 - 5 - 4)) {
        LOG_ERR("Packet too long");
        return -ENOSYS;
    }

    if(msg->len == 0){
        LOG_ERR("Message length 0");
        return -ENOSYS;
    }

    LOG_INF("Flushing TX buffer");
    // Flush TX buffer (TX_FLUSH)
    int res = i2c_reg_write_byte_dt(&cfg->i2c, REG_CONTROL0, 0b01000000);
    if (res != 0) { return -EIO; }

    uint8_t sop[5] = {TX_TOKEN_SOP1, TX_TOKEN_SOP1, TX_TOKEN_SOP1, TX_TOKEN_SOP2,
                      TX_TOKEN_PACKSYM | ((msg->len + 2) & 0b00011111)};
    //LOG_HEXDUMP_WRN(sop, 5, "SOP");
    res = i2c_burst_write_dt(&cfg->i2c, REG_FIFO, sop, sizeof(sop));
    if (res != 0) { return -EIO; }

    uint8_t header[2] = {msg->header.raw_value & 0xFF, msg->header.raw_value >> 8};
    //LOG_HEXDUMP_WRN(header, 2, "header");
    res = i2c_burst_write_dt(&cfg->i2c, REG_FIFO, header, sizeof(header));
    if (res != 0) { return -EIO; }

    //LOG_HEXDUMP_WRN(msg->data, msg->len, "data");
    res = i2c_burst_write_dt(&cfg->i2c, REG_FIFO, msg->data, msg->len);
    if (res != 0) { return -EIO; }

    uint8_t eop[4] = {TX_TOKEN_JAM_CRC, TX_TOKEN_EOP, TX_TOKEN_TXOFF, TX_TOKEN_TXON};
    //LOG_HEXDUMP_WRN(eop, 4, "EOP");
    res = i2c_burst_write_dt(&cfg->i2c, REG_FIFO, eop, sizeof(eop));
    if (res != 0) { return -EIO; }

    // TODO: Do this when goodcrc received data->alert_info.handler(dev, data->alert_info.data, TCPC_ALERT_TRANSMIT_MSG_SUCCESS);

    LOG_ERR("TRANSMIT DONE");
    return 0;
}

// required == assumed by assertion, optional == results in ENOSYS
static const struct tcpc_driver_api fusb302b_tcpc_driver_api = {
        .init = fusb302_setup,
        .get_cc = fusb302b_get_cc,
        .select_rp_value=NULL, // Optional
        .get_rp_value = NULL,// Optional
        .set_cc = fusb302b_set_cc,
        .set_vconn_discharge_cb = fusb302b_set_vconn_discharge_cb,
        .set_vconn_cb = fusb302b_set_vconn_cb,
        .vconn_discharge = NULL,// Optional
        .set_vconn = NULL, // Optional
        .set_roles = NULL, // Optional
        .receive_data = fusb302b_receive_data, // Optional
        .is_rx_pending_msg = fusb302b_is_rx_pending_msg, // Optional
        .set_rx_enable = fusb302b_set_rx_enable, // Optional
        .set_cc_polarity = fusb302b_set_cc_polarity,
        .transmit_data = fusb302b_transmit_data, // Optional
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
        // The Power Delivery Protocol Layer code will call this and register its callback.
        //  We should notify it "when messages are received, transmitted, etc".
        .set_alert_handler_cb=fusb302b_set_alert_handler_cb, // Required
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
        /* level = */ POST_KERNEL, \
        CONFIG_USBC_TCPC_INIT_PRIORITY, \
        &fusb302b_tcpc_driver_api \
    );

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) > 0, "No compatible FUSB302B instance found");

DT_INST_FOREACH_STATUS_OKAY(FUSB302B_DEFINE)
