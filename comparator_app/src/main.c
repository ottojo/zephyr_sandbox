#include "autoconf.h"
#include <stdbool.h>
#include <zephyr/kernel.h>

#ifndef CONFIG_COMPARATOR_STM32
#error stm32 comparator not found!
#endif

int main() {
  while (true) {
    k_msleep(100);
  }
}
