/* SPDX-License-Identifier: MIT */
/* The host's part of the RAM log's boundary (common/log/log_ram_os.h) for
 * tests/log_ram.c: the ring in ordinary memory, with a pointer to it so
 * that the test can play a restart (keep the ring, call
 * log_ram_initialize() again) or a power-on (spoil its header); the
 * interrupt mask counted; the clock and the attach hook the test's.
 */
#ifndef TEST_LOG_RAM_OS_IMPL_H
#define TEST_LOG_RAM_OS_IMPL_H

#include <assert.h>
#include <stdint.h>

struct retained_log;

/* The declarator ends the ring's static declaration and declares the
 * test's pointer to it.
 */
#define LOG_RAM_OS_RETAINED(name) name; struct retained_log *const log_ram_test_ring = &name

typedef int log_ram_os_irqstate_t;

extern int log_ram_test_masked;   /* nesting depth of the mask */
extern int log_ram_test_saves;    /* log_ram_os_irq_save() calls */
extern int log_ram_test_attached; /* log_ram_os_attach() calls */
extern uint64_t log_ram_test_now_us;

static inline log_ram_os_irqstate_t log_ram_os_irq_save(void)
{
	log_ram_test_saves++;
	return log_ram_test_masked++;
}

static inline void log_ram_os_irq_restore(log_ram_os_irqstate_t state)
{
	assert(log_ram_test_masked == state + 1);
	log_ram_test_masked = state;
}

static inline uint64_t log_ram_os_uptime_us(void)
{
	return log_ram_test_now_us;
}

static inline void log_ram_os_attach(void)
{
	assert(log_ram_test_masked == 0);
	log_ram_test_attached++;
}

#endif /* TEST_LOG_RAM_OS_IMPL_H */
