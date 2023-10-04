#include <posix_board_if.h>
#include <zephyr/drivers/emul_usbc_vbus.h>
#include <zephyr/drivers/usb_c/fusb302b.h>
#include <zephyr/drivers/usb_c/usbc_vbus.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(main);

int main() {
  const struct device *const fusb_dev = DEVICE_DT_GET(DT_NODELABEL(fusb));
  const struct device *const vbus_dev = DEVICE_DT_GET(DT_NODELABEL(fvbus));

  const struct emul *vbus_emul = EMUL_DT_GET(DT_NODELABEL(fvbus));

  __ASSERT_NO_MSG(device_is_ready(fusb_dev));
  __ASSERT_NO_MSG(device_is_ready(vbus_dev));

  emul_usbc_vbus_set_vbus_voltage(vbus_emul, 5000);

  usbc_vbus_enable(vbus_dev, true);
  int meas_result = 0;
  int res = usbc_vbus_measure(vbus_dev, &meas_result);
  __ASSERT(res == 0, "Error during measure: %d", res);
  LOG_INF("VBUS measured at %dmV", meas_result);
  __ASSERT(meas_result < 5250 && meas_result > 4750,
           "VBUS measurement failed, measured %dmV", meas_result);

  posix_exit(0);
  return 0;
}
