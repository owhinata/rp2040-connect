/* SPDX-License-Identifier: MIT */
/* NuttX's sleeps for tests/nina_os.c: on the test's clock. */

#ifndef TEST_NINA_OS_NUTTX_SCHED_H
#define TEST_NINA_OS_NUTTX_SCHED_H

void nxsched_usleep(unsigned int usec);
void nxsched_msleep(unsigned int msec);

#endif /* TEST_NINA_OS_NUTTX_SCHED_H */
