/* SPDX-License-Identifier: Apache-2.0 */
/* NuttX's mutex_t (<nuttx/mutex.h>), defined by tests/nina_netdev.c. */

#ifndef TEST_NINA_NETDEV_NUTTX_MUTEX_H
#define TEST_NINA_NETDEV_NUTTX_MUTEX_H

typedef struct
{
  int locked;
} mutex_t;

#define NXMUTEX_INITIALIZER {0}

int nxmutex_lock(mutex_t *m);
int nxmutex_unlock(mutex_t *m);

#endif /* TEST_NINA_NETDEV_NUTTX_MUTEX_H */
