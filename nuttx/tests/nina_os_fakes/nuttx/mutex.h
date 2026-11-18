/* SPDX-License-Identifier: MIT */
/* NuttX's mutexes for tests/nina_os.c (one thread): counts. */

#ifndef TEST_NINA_OS_NUTTX_MUTEX_H
#define TEST_NINA_OS_NUTTX_MUTEX_H

typedef struct
{
  int depth;
} rmutex_t;

typedef struct
{
  int locked;
} mutex_t;

#define NXRMUTEX_INITIALIZER {0}
#define NXMUTEX_INITIALIZER  {0}

int nxrmutex_lock(rmutex_t *m);
int nxrmutex_unlock(rmutex_t *m);
int nxmutex_lock(mutex_t *m);
int nxmutex_unlock(mutex_t *m);

#endif /* TEST_NINA_OS_NUTTX_MUTEX_H */
