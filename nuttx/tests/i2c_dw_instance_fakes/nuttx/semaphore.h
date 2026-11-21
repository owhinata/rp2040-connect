/* SPDX-License-Identifier: MIT */
/* <nuttx/semaphore.h> for the host test of the I2C core's NuttX binding
 * (tests/i2c_dw_instance.c): the nxsem_*() calls that i2c_dw_os_impl.h and
 * i2c_dw_instance.h use, on the host's POSIX semaphores (ticks of 1 ms,
 * nuttx/clock.h here).
 */
#ifndef TEST_I2C_DW_NUTTX_SEMAPHORE_H
#define TEST_I2C_DW_NUTTX_SEMAPHORE_H
#include <errno.h>
#include <semaphore.h>
#include <stdint.h>
#include <time.h>
static inline int nxsem_init(sem_t *sem, int pshared, unsigned int value)
{
	return sem_init(sem, pshared, value) == 0 ? 0 : -errno;
}
static inline int nxsem_post(sem_t *sem)
{
	return sem_post(sem) == 0 ? 0 : -errno;
}
static inline int nxsem_wait_uninterruptible(sem_t *sem)
{
	while (sem_wait(sem) != 0) {
	}
	return 0;
}
static inline int nxsem_tickwait_uninterruptible(sem_t *sem, uint32_t ticks)
{
	struct timespec ts;

	clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_sec += ticks / 1000;
	ts.tv_nsec += (long)(ticks % 1000) * 1000000L;
	if (ts.tv_nsec >= 1000000000L) {
		ts.tv_sec++;
		ts.tv_nsec -= 1000000000L;
	}
	while (sem_timedwait(sem, &ts) != 0) {
		if (errno == ETIMEDOUT) {
			return -ETIMEDOUT;
		}
	}
	return 0;
}
static inline int nxsem_reset(sem_t *sem, int16_t count)
{
	while (sem_trywait(sem) == 0) {
	}
	for (int16_t i = 0; i < count; i++) {
		sem_post(sem);
	}
	return 0;
}
#endif
