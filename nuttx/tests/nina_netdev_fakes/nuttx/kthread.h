/* SPDX-License-Identifier: Apache-2.0 */
/* NuttX's kthread_create_with_stack() (<nuttx/kthread.h>), defined by
 * tests/nina_netdev.c.
 */

#ifndef TEST_NINA_NETDEV_NUTTX_KTHREAD_H
#define TEST_NINA_NETDEV_NUTTX_KTHREAD_H

typedef int (*main_t)(int argc, char *argv[]);

int kthread_create_with_stack(const char *name, int priority, void *stack_addr, int stack_size,
                              main_t entry, char * const argv[]);

#endif /* TEST_NINA_NETDEV_NUTTX_KTHREAD_H */
