/* SPDX-License-Identifier: Apache-2.0 */
#include "imu_cpu_profile.h"
#ifdef CONFIG_IMU_CPU_PROFILE_IRQS
#include <cmsis_core.h>
#include <hardware/platform_defs.h>
#include <hardware/regs/i2c.h>
#include <zephyr/sys/sys_io.h>
#endif
#include <inttypes.h>
#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#define PROFILE_THREADS       32
#define PROFILE_EXCEPTIONS    48
#define PROFILE_SAMPLE_PERIOD 61U

struct thread_entry {
	k_tid_t id;
	char name[24];
	uint64_t cycles;
};
#ifdef CONFIG_IMU_CPU_PROFILE_IRQS
struct irq_entry {
	uint64_t cycles;
	uint64_t idle_cycles;
	uint32_t count;
	uint32_t samples;
	uint32_t started;
	bool sampling;
	bool interrupted_idle;
};
static struct irq_entry irq_data[PROFILE_EXCEPTIONS];
/* Sampled masked I2C interrupt conditions: neither, TX only, RX only, both. */
static uint32_t i2c_status_samples[4];
static uint32_t i2c_tx_only_rx_empty;
static uint32_t irq_overflow;
static volatile bool active;
static k_tid_t idle_thread;

/* Each vector samples one in 61 invocations. The same exception cannot
 * preempt itself on Cortex-M. Sampled times include any nested exceptions.
 */
void sys_trace_isr_enter(void)
{
	uint32_t vector;
	struct irq_entry *entry;

	if (!active) {
		return;
	}
	vector = __get_IPSR();
	if (vector >= PROFILE_EXCEPTIONS) {
		irq_overflow++;
		return;
	}
	entry = &irq_data[vector];
	entry->count++;
	entry->sampling = (entry->count % PROFILE_SAMPLE_PERIOD) == 1;
	if (entry->sampling) {
		if (vector == DT_IRQN(DT_NODELABEL(i2c0)) + 16) {
			uint32_t status = sys_read32(DT_REG_ADDR(DT_NODELABEL(i2c0)) +
						     I2C_IC_INTR_STAT_OFFSET);
			unsigned int category =
				(status & I2C_IC_INTR_STAT_R_TX_EMPTY_BITS ? 1U : 0U) |
				(status & I2C_IC_INTR_STAT_R_RX_FULL_BITS ? 2U : 0U);
			i2c_status_samples[category]++;
			if (category == 1 && sys_read32(DT_REG_ADDR(DT_NODELABEL(i2c0)) +
							I2C_IC_RXFLR_OFFSET) == 0) {
				i2c_tx_only_rx_empty++;
			}
		}
		entry->interrupted_idle = k_current_get() == idle_thread;
		entry->started = k_cycle_get_32();
	}
}

void sys_trace_isr_exit(void)
{
	uint32_t vector;
	struct irq_entry *entry;
	uint32_t elapsed;

	if (!active) {
		return;
	}
	vector = __get_IPSR();
	if (vector >= PROFILE_EXCEPTIONS) {
		return;
	}
	entry = &irq_data[vector];
	if (entry->sampling) {
		elapsed = k_cycle_get_32() - entry->started;
		entry->cycles += elapsed;
		entry->samples++;
		if (entry->interrupted_idle) {
			entry->idle_cycles += elapsed;
		}
		entry->sampling = false;
	}
}

#endif /* CONFIG_IMU_CPU_PROFILE_IRQS */

static struct thread_entry before[PROFILE_THREADS], after[PROFILE_THREADS];
static unsigned int before_count, after_count, thread_overflow;

struct thread_capture {
	struct thread_entry *entries;
	unsigned int count;
};
static void capture_thread(const struct k_thread *thread, void *arg)
{
	struct thread_capture *capture = arg;
	k_thread_runtime_stats_t stats;
	const char *name;
	struct thread_entry *entry;

	if (capture->count >= PROFILE_THREADS) {
		thread_overflow++;
		return;
	}
	if (k_thread_runtime_stats_get((k_tid_t)thread, &stats) != 0) {
		thread_overflow++;
		return;
	}
	entry = &capture->entries[capture->count++];
	entry->id = (k_tid_t)thread;
	entry->cycles = stats.execution_cycles;
	name = k_thread_name_get((k_tid_t)thread);
	if (name) {
		strncpy(entry->name, name, sizeof(entry->name) - 1);
		entry->name[sizeof(entry->name) - 1] = '\0';
	} else {
		entry->name[0] = '\0';
	}
}
static unsigned int capture_threads(struct thread_entry *entries)
{
	struct thread_capture capture = {.entries = entries};
	k_thread_foreach(capture_thread, &capture);
	return capture.count;
}
void imu_cpu_profile_start(void)
{
#ifdef CONFIG_IMU_CPU_PROFILE_IRQS
	unsigned int key;
#endif
	thread_overflow = 0;
	before_count = capture_threads(before);
#ifdef CONFIG_IMU_CPU_PROFILE_IRQS
	idle_thread = NULL;
	for (unsigned int i = 0; i < before_count; i++) {
		if (!strcmp(before[i].name, "idle")) {
			idle_thread = before[i].id;
			break;
		}
	}
	key = irq_lock();
	memset(irq_data, 0, sizeof(irq_data));
	memset(i2c_status_samples, 0, sizeof(i2c_status_samples));
	i2c_tx_only_rx_empty = 0;
	irq_overflow = 0;
	active = true;
	irq_unlock(key);
#endif
}
void imu_cpu_profile_stop(void)
{
#ifdef CONFIG_IMU_CPU_PROFILE_IRQS
	unsigned int key = irq_lock();
	active = false;
	irq_unlock(key);
#endif
	after_count = capture_threads(after);
}
#ifdef CONFIG_IMU_CPU_PROFILE_IRQS
static const char *irq_label(unsigned int irq)
{
	if (irq == 15) {
		return "systick";
	}
	if (irq < 16) {
		return "exception";
	}
	irq -= 16;
	if (irq == DT_IRQN(DT_NODELABEL(i2c0))) {
		return "i2c0";
	}
	if (irq == DT_IRQN(DT_NODELABEL(gpio0_map))) {
		return "gpio0";
	}
	return "other";
}
#endif
void imu_cpu_profile_print(const struct shell *shell, uint64_t total_cycles, uint64_t busy_cycles)
{
	if (!total_cycles) {
		shell_print(shell, "CPU profile unavailable");
		return;
	}
#ifdef CONFIG_IMU_CPU_PROFILE_IRQS
	if (idle_thread) {
		uint64_t idle_irq_cycles = 0;
		for (unsigned int i = 0; i < PROFILE_EXCEPTIONS; i++) {
			if (irq_data[i].samples) {
				idle_irq_cycles += irq_data[i].idle_cycles * irq_data[i].count /
						   irq_data[i].samples;
			}
		}
		uint64_t pct = (busy_cycles + idle_irq_cycles) * 10000ULL / total_cycles;
		shell_print(shell,
			    "busy_with_sampled_idle_irq=%" PRIu64 ".%02" PRIu64
			    "%% (estimate; nested IRQ may overlap)",
			    pct / 100, pct % 100);
	} else {
		shell_print(shell, "idle thread not identified; corrected busy unavailable");
	}
#else
	ARG_UNUSED(busy_cycles);
#endif
	shell_print(shell, "CPU profile: scheduler thread time (includes interrupts)");
	for (unsigned int i = 0; i < after_count; i++) {
		for (unsigned int j = 0; j < before_count; j++) {
			if (after[i].id == before[j].id && after[i].cycles >= before[j].cycles) {
				uint64_t cycles = after[i].cycles - before[j].cycles;
				uint64_t pct = cycles * 10000ULL / total_cycles;
				if (pct) {
					shell_print(shell,
						    "thread %s (%p): %" PRIu64 ".%02" PRIu64 "%%",
						    after[i].name[0] ? after[i].name : "unnamed",
						    (void *)after[i].id, pct / 100, pct % 100);
				}
				break;
			}
		}
	}
#ifdef CONFIG_IMU_CPU_PROFILE_IRQS
	shell_print(shell, "Sampled interrupts: 1/61, estimated inclusive time; overlaps threads");
	for (unsigned int i = 0; i < PROFILE_EXCEPTIONS; i++) {
		if (irq_data[i].count) {
			uint64_t cycles = irq_data[i].samples
						  ? irq_data[i].cycles * irq_data[i].count /
							    irq_data[i].samples
						  : 0;
			uint64_t pct = cycles * 10000ULL / total_cycles;
			shell_print(shell,
				    "vector %u (%s): ~%" PRIu64 ".%02" PRIu64
				    "%% count=%u samples=%u",
				    i, irq_label(i), pct / 100, pct % 100, irq_data[i].count,
				    irq_data[i].samples);
		}
	}
	shell_print(shell, "i2c_status_samples tx_only=%u rx_only=%u both=%u neither=%u",
		    i2c_status_samples[1], i2c_status_samples[2], i2c_status_samples[3],
		    i2c_status_samples[0]);
	shell_print(shell, "i2c_tx_only_samples_rx_fifo_empty=%u", i2c_tx_only_rx_empty);
	shell_print(shell, "profile_overflow threads=%u irq=%u", thread_overflow, irq_overflow);
#else
	shell_print(shell, "profile_overflow threads=%u; interrupts not measured", thread_overflow);
#endif
}
