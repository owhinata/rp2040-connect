/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of the compatibility layer's k_mutex on NuttX
 * (nuttx/drivers/compat/zephyr_mutex.c), built with __NuttX__ on a model
 * of NuttX 13.1.0's recursive mutex (tests/zephyr_mutex_fakes/nuttx/
 * mutex.h: an all-zero mutex is held by thread 0).
 *
 * Zephyr's k_mutex is unlocked when all zero (kernel/mutex.c: lock_count
 * 0), and Zephyr's MQTT library relies on it: mqtt_abort() locks the
 * client's mutex, which only mqtt_client_init() initializes, and
 * imu_raw_mqtt.c calls it after an attempt that ended before
 * mqtt_client_init() (no default gateway yet: -ENETUNREACH).  On NuttX's
 * all-zero mutex that lock waited for ever: the transport thread stopped
 * before counting the failure (T27).  Checked here: a mutex left all zero
 * locks at once (K_FOREVER, K_NO_WAIT, a timeout), recursively, unlocks,
 * and is then an ordinary mutex for other threads; K_MUTEX_DEFINE() and
 * k_mutex_init(); a mutex held by another thread gives -EBUSY and -EAGAIN.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>

uint32_t fake_tid = 11;
int fake_waits_forever, fake_tick_waits, fake_irq_saved, fake_sched_locked;

K_MUTEX_DEFINE(defined_lock);

/* What struct mqtt_client holds before mqtt_client_init(): zeros. */
static struct {
	int state;
	struct k_mutex mutex;
} zeroed_client;

static void as_thread(uint32_t tid)
{
	fake_tid = tid;
}

int main(void)
{
	struct k_mutex local;

	/* K_MUTEX_DEFINE(): unlocked, recursive for its owner. */
	assert(k_mutex_lock(&defined_lock, K_FOREVER) == 0);
	assert(k_mutex_lock(&defined_lock, K_FOREVER) == 0);
	assert(k_mutex_unlock(&defined_lock) == 0);
	assert(k_mutex_unlock(&defined_lock) == 0);
	assert(fake_waits_forever == 0);

	/* All zero, as the MQTT client's mutex before mqtt_client_init():
	 * the first lock takes it at once (Zephyr: lock_count 0). */
	assert(k_mutex_lock(&zeroed_client.mutex, K_FOREVER) == 0);
	assert(fake_waits_forever == 0);
	assert(k_mutex_lock(&zeroed_client.mutex, K_FOREVER) == 0);
	assert(k_mutex_unlock(&zeroed_client.mutex) == 0);
	assert(k_mutex_unlock(&zeroed_client.mutex) == 0);
	assert(fake_irq_saved == 0 && fake_sched_locked == 0);

	/* Then an ordinary mutex: another thread takes it, the first one
	 * gets -EBUSY and -EAGAIN while it is held. */
	as_thread(12);
	assert(k_mutex_lock(&zeroed_client.mutex, K_FOREVER) == 0);
	as_thread(11);
	assert(k_mutex_lock(&zeroed_client.mutex, K_NO_WAIT) == -EBUSY);
	assert(k_mutex_lock(&zeroed_client.mutex, K_MSEC(5)) == -EAGAIN && fake_tick_waits == 1);
	as_thread(12);
	assert(k_mutex_unlock(&zeroed_client.mutex) == 0);
	as_thread(11);
	assert(k_mutex_lock(&zeroed_client.mutex, K_NO_WAIT) == 0);
	assert(k_mutex_unlock(&zeroed_client.mutex) == 0);

	/* All zero, first taken without waiting or with a timeout. */
	memset(&local, 0, sizeof(local));
	assert(k_mutex_lock(&local, K_NO_WAIT) == 0);
	assert(k_mutex_unlock(&local) == 0);
	memset(&local, 0, sizeof(local));
	assert(k_mutex_lock(&local, K_MSEC(5)) == 0 && fake_tick_waits == 1);
	assert(k_mutex_unlock(&local) == 0);

	/* k_mutex_init() on any contents: unlocked. */
	memset(&local, 0, sizeof(local));
	assert(k_mutex_init(&local) == 0);
	assert(k_mutex_lock(&local, K_FOREVER) == 0);
	assert(k_mutex_unlock(&local) == 0);

	assert(fake_waits_forever == 0);
	assert(fake_irq_saved == 0 && fake_sched_locked == 0);
	printf("zephyr_mutex: ok\n");
	return 0;
}
