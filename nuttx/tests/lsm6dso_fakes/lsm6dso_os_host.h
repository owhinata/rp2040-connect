/* SPDX-License-Identifier: Apache-2.0 */
/* The kernel object types of lsm6dso_os.h for the host test
 * (nuttx/tests/lsm6dso_driver.c implements the functions).
 */
#ifndef TEST_FAKE_LSM6DSO_OS_HOST_H
#define TEST_FAKE_LSM6DSO_OS_HOST_H
#include <stdint.h>
typedef struct { int depth; } lsm6dso_mutex_t;
typedef struct { int locked; } lsm6dso_spinlock_t;
typedef int lsm6dso_spinlock_key_t;
#endif
