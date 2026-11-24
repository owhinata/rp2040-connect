/* SPDX-License-Identifier: MIT */
/* The OS part of the port's priorities (common/compat/include/priorities.h) for the
 * host check of the generic kernel part (zephyr_kernel_os_impl.h here):
 * larger is more urgent, as the generic part's z_os_prio_t.
 */

#ifndef COMMON_TESTS_KERNEL_GENERIC_HOST_PRIORITIES_OS_H
#define COMMON_TESTS_KERNEL_GENERIC_HOST_PRIORITIES_OS_H

#define APP_PRIO_FROM_ZEPHYR(p) (20 - (p))

#endif /* COMMON_TESTS_KERNEL_GENERIC_HOST_PRIORITIES_OS_H */
