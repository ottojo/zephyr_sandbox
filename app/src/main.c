#include <stdbool.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/usb_c/fusb302b.h>
#include <zephyr/drivers/usb_c/usbc_vbus.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb_c/usbc.h>

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

const struct device *const comp1 = DEVICE_DT_GET(DT_NODELABEL(comp1));
const struct device *const comp2 = DEVICE_DT_GET(DT_NODELABEL(comp2));

uint32_t next_pdo = 1;

static void display_pdo(const int idx, const uint32_t pdo_value) {
	union pd_fixed_supply_pdo_source pdo;

	/* Default to fixed supply pdo source until type is detected */
	pdo.raw_value = pdo_value;

	LOG_INF("PDO %d:", idx);
	switch (pdo.type) {
		case PDO_FIXED: {
			LOG_INF("\tType:              FIXED");
			// LOG_INF("\tCurrent:           %d",
			//         PD_CONVERT_FIXED_PDO_CURRENT_TO_MA(pdo.max_current));
			LOG_INF("\tVoltage:           %d", PD_CONVERT_FIXED_PDO_VOLTAGE_TO_MV(pdo.voltage));

		} break;
		case PDO_BATTERY: {
			union pd_battery_supply_pdo_source batt_pdo;

			batt_pdo.raw_value = pdo_value;
			LOG_INF("\tType:              BATTERY");
			LOG_INF("\tMin Voltage: %d", PD_CONVERT_BATTERY_PDO_VOLTAGE_TO_MV(batt_pdo.min_voltage));
			LOG_INF("\tMax Voltage: %d", PD_CONVERT_BATTERY_PDO_VOLTAGE_TO_MV(batt_pdo.max_voltage));
			LOG_INF("\tMax Power:   %d", PD_CONVERT_BATTERY_PDO_POWER_TO_MW(batt_pdo.max_power));
		} break;
		case PDO_VARIABLE: {
			union pd_variable_supply_pdo_source var_pdo;

			var_pdo.raw_value = pdo_value;
			LOG_INF("\tType:        VARIABLE");
			LOG_INF("\tMin Voltage: %d", PD_CONVERT_VARIABLE_PDO_VOLTAGE_TO_MV(var_pdo.min_voltage));
			LOG_INF("\tMax Voltage: %d", PD_CONVERT_VARIABLE_PDO_VOLTAGE_TO_MV(var_pdo.max_voltage));
			LOG_INF("\tMax Current: %d", PD_CONVERT_VARIABLE_PDO_CURRENT_TO_MA(var_pdo.max_current));
		} break;
		case PDO_AUGMENTED: {
			union pd_augmented_supply_pdo_source aug_pdo;

			aug_pdo.raw_value = pdo_value;
			LOG_INF("\tType:              AUGMENTED");
			LOG_INF("\tMin Voltage:       %d", PD_CONVERT_AUGMENTED_PDO_VOLTAGE_TO_MV(aug_pdo.min_voltage));
			LOG_INF("\tMax Voltage:       %d", PD_CONVERT_AUGMENTED_PDO_VOLTAGE_TO_MV(aug_pdo.max_voltage));
			LOG_INF("\tMax Current:       %d", PD_CONVERT_AUGMENTED_PDO_CURRENT_TO_MA(aug_pdo.max_current));
			LOG_INF("\tPPS Power Limited: %d", aug_pdo.pps_power_limited);
		} break;
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

	rdo.fixed.object_pos = next_pdo;

	LOG_INF("Requesting PDO %d", next_pdo);

	return rdo.raw_value;
}

void handle_src_caps(const struct device *dev, const uint32_t *pdos, const int num_pdos) {

	if (num_pdos == 0) { return; }
	for (int i = 0; i < num_pdos; i++) {
		//display_pdo(i, pdos[i]);
		union pd_fixed_supply_pdo_source pdo;
		/* Default to fixed supply pdo source until type is detected */
		pdo.raw_value = pdos[i];
		/*if (pdo.voltage == PD_CONVERT_MV_TO_FIXED_PDO_VOLTAGE(9000)) {
			LOG_WRN("Selecting PDO for use!");
			pdo_pos = i + 1;
		}
		 */
	}
	next_pdo++;
	if (next_pdo > (num_pdos)) { next_pdo = 1; }
	LOG_INF("Received %d PDOs, next is %d!", num_pdos, next_pdo);
}

const char *usbc_policy_notify_to_str(const enum usbc_policy_notify_t policy_notify) {
	switch (policy_notify) {
		case MSG_ACCEPT_RECEIVED:
			return "MSG_ACCEPT_RECEIVED";
		case MSG_REJECTED_RECEIVED:
			return "MSG_REJECTED_RECEIVED";
		case MSG_DISCARDED:
			return "MSG_DISCARDED";
		case MSG_NOT_SUPPORTED_RECEIVED:
			return "MSG_NOT_SUPPORTED_RECEIVED";
		case DATA_ROLE_IS_UFP:
			return "DATA_ROLE_IS_UFP";
		case DATA_ROLE_IS_DFP:
			return "DATA_ROLE_IS_DFP";
		case PD_CONNECTED:
			return "PD_CONNECTED";
		case NOT_PD_CONNECTED:
			return "NOT_PD_CONNECTED";
		case TRANSITION_PS:
			return "TRANSITION_PS";
		case PORT_PARTNER_NOT_RESPONSIVE:
			return "PORT_PARTNER_NOT_RESPONSIVE";
		case PROTOCOL_ERROR:
			return "PROTOCOL_ERROR";
		case SNK_TRANSITION_TO_DEFAULT:
			return "SNK_TRANSITION_TO_DEFAULT";
		case HARD_RESET_RECEIVED:
			return "HARD_RESET_RECEIVED";
		case POWER_CHANGE_0A0:
			return "POWER_CHANGE_0A0";
		case POWER_CHANGE_DEF:
			return "POWER_CHANGE_DEF";
		case POWER_CHANGE_1A5:
			return "POWER_CHANGE_1A5";
		case POWER_CHANGE_3A0:
			return "POWER_CHANGE_3A0";
		case SENDER_RESPONSE_TIMEOUT:
			return "SENDER_RESPONSE_TIMEOUT";
		case SOURCE_CAPABILITIES_RECEIVED:
			return "SOURCE_CAPABILITIES_RECEIVED";
	}
	return "<invalid enum usbc_policy_notify_t>";
}

void notify_cb(const struct device *dev, const enum usbc_policy_notify_t policy_notify) {
	switch (policy_notify) {
		case TRANSITION_PS:
			LOG_INF("TRANSITION_PS");
			break;
		case PORT_PARTNER_NOT_RESPONSIVE:
			LOG_ERR("Port partner unresponsive.");
			break;
		default:
			LOG_WRN("Unhandled notification: %s", usbc_policy_notify_to_str(policy_notify));
			break;
	}
}

int main() {
	int ret;

	if (!gpio_is_ready_dt(&led)) { return 1; }

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) { return 1; }

	if (!device_is_ready(comp1)) {
		LOG_ERR("COMP1 not init");
		while (true) { k_msleep(100); }
	}

	if (!device_is_ready(comp2)) {
		LOG_ERR("COMP2 not init");
		while (true) { k_msleep(100); }
	}

	usbc_set_policy_cb_set_src_cap(port1, handle_src_caps);
	usbc_set_policy_cb_get_rdo(port1, policy_cb_get_rdo);
	usbc_set_policy_cb_notify(port1, notify_cb);


	LOG_INF("Starting USB-C");
	usbc_start(port1);
	k_sleep(K_MSEC(1000));

	while (true) {
		gpio_pin_toggle_dt(&led);
		k_sleep(K_MSEC(1000));
		usbc_request(port1, REQUEST_PE_GET_SRC_CAPS);
	}
}
