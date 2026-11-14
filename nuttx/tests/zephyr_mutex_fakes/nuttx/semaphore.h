/* SPDX-License-Identifier: Apache-2.0 */
/* The sem_t of <nuttx/semaphore.h> that <zephyr/kernel.h> names (struct
 * k_sem), for the host test of zephyr_mutex.c. */
#ifndef TEST_ZMUTEX_NUTTX_SEMAPHORE_H
#define TEST_ZMUTEX_NUTTX_SEMAPHORE_H
typedef struct { int semcount; } sem_t;
#define SEM_INITIALIZER(c) { (c) }
#define SEM_VALUE_MAX 32767
#endif
