/* SPDX-License-Identifier: MIT */
/* The OS part of the port's priorities (common/compat/priorities.h) for
 * host tests: the NuttX build's mapping of Zephyr's priorities (118 - p,
 * nuttx/drivers/include/priorities_os.h), as K_PRIO_PREEMPT() of the fake
 * kernel (zephyr_os.h).
 */

#ifndef ZEPHYR_FAKES_PRIORITIES_OS_H
#define ZEPHYR_FAKES_PRIORITIES_OS_H

#define APP_PRIO_FROM_ZEPHYR(p) (118 - (p))

#endif /* ZEPHYR_FAKES_PRIORITIES_OS_H */
