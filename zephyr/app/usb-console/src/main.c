#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <pico/bootrom.h>
#include "app_main.h"
#include "nina_passthrough.h"

/* "kernel ps": each thread's share of the CPU time since the previous
 * "kernel ps" (since the start for the first), from any shell, and the use
 * of its stack, then the interrupt stack, as Thread Analyzer
 * (subsys/debug/thread_analyzer) shows them.  Thread Analyzer's own CPU
 * figure is the share since the start, and its callback does not say which
 * thread a row is, so the threads are read here: the runtime statistics
 * (CONFIG_SCHED_THREAD_USAGE, cycles counted at each switch; interrupt
 * handlers count in the thread they interrupted) of this list less those of
 * the previous one, for each thread by its handle
 * (common/shell/ps_interval.h; nothing runs between two lists).  The share
 * is of all the cycles of the interval (execution_cycles of
 * k_thread_runtime_stats_all_get(): non-idle and idle), so the rows, the
 * idle thread's included, add up to about 100%.
 */
#define PS_INTERVAL_TASKS_MAX 32
#include "ps_interval.h"

/* For z_stack_space_get(), as Thread Analyzer and "kernel thread stacks". */
#include <kernel_internal.h>

K_KERNEL_STACK_ARRAY_DECLARE(z_interrupt_stacks, CONFIG_MP_MAX_NUM_CPUS, CONFIG_ISR_STACK_SIZE);

struct ps_thread {
	k_tid_t tid;
	char name[CONFIG_THREAD_MAX_NAME_LEN];
	size_t stack_size;
	size_t stack_unused;
	int stack_err;
	uint64_t cycles;
};

/* Under ps_lock. */
K_MUTEX_DEFINE(ps_lock);
static struct ps_thread ps_threads[PS_INTERVAL_TASKS_MAX];
static size_t ps_count;
static size_t ps_more;
static struct ps_interval ps_interval;

static void ps_read_thread(const struct k_thread *cthread, void *user_data)
{
	struct k_thread *thread = (struct k_thread *)cthread;
	struct ps_thread *t;
	k_thread_runtime_stats_t usage;
	const char *name;

	ARG_UNUSED(user_data);
	if (ps_count >= ARRAY_SIZE(ps_threads)) {
		ps_more++;
		return;
	}
	t = &ps_threads[ps_count++];
	t->tid = thread;
	name = k_thread_name_get(thread);
	if (name != NULL && name[0] != '\0') {
		strncpy(t->name, name, sizeof(t->name) - 1);
		t->name[sizeof(t->name) - 1] = '\0';
	} else {
		snprintk(t->name, sizeof(t->name), "%p", (void *)thread);
	}
	t->stack_size = thread->stack_info.size;
	t->stack_err = k_thread_stack_space_get(thread, &t->stack_unused);
	if (t->stack_err != 0) {
		t->stack_unused = 0;
	}
	t->cycles = 0;
	if (k_thread_runtime_stats_get(thread, &usage) == 0) {
		t->cycles = usage.execution_cycles;
	}
}

static int cmd_ps(const struct shell *sh, size_t argc, char **argv)
{
	k_thread_runtime_stats_t all = {0};
	uint64_t elapsed;
	uint32_t hz = sys_clock_hw_cycles_per_sec();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	k_mutex_lock(&ps_lock, K_FOREVER);
	ps_count = 0;
	ps_more = 0;
	k_thread_foreach_unlocked(ps_read_thread, NULL);
	/* After the threads: the interval is not shorter than their times. */
	(void)k_thread_runtime_stats_all_get(&all);

	elapsed = ps_interval_begin(&ps_interval, all.execution_cycles);
	for (size_t i = 0; i < ps_count; i++) {
		struct ps_thread *t = &ps_threads[i];
		uint64_t run = ps_interval_task(&ps_interval, (uintptr_t)t->tid, 0, t->cycles);
		unsigned int permille = ps_interval_permille(run, elapsed);

		if (t->stack_err != 0) {
			shell_print(sh, " %-20s: unable to get stack space (%d)", t->name,
				    t->stack_err);
		}
		shell_print(sh, "%-20s CPU: %3u.%u%%  STACK: %zu / %zu bytes", t->name,
			    permille / 10, permille % 10, t->stack_size - t->stack_unused,
			    t->stack_size);
	}
	ps_interval_end(&ps_interval);
	if (ps_more != 0) {
		shell_print(sh, "(%zu more threads not shown)", ps_more);
	}

	for (unsigned int i = 0; i < arch_num_cpus(); i++) {
		const uint8_t *buf = K_KERNEL_STACK_BUFFER(z_interrupt_stacks[i]);
		size_t size = K_KERNEL_STACK_SIZEOF(z_interrupt_stacks[i]);
		size_t unused;

		if (z_stack_space_get(buf, size, &unused) == 0) {
			shell_print(sh, "ISR%-17u CPU: %6s  STACK: %zu / %zu bytes", i, "-",
				    size - unused, size);
		}
	}
	shell_print(sh, "CPU: share of the %llu.%03llu s since the previous \"kernel ps\"",
		    (unsigned long long)(elapsed / hz),
		    (unsigned long long)(elapsed % hz * 1000u / hz));
	k_mutex_unlock(&ps_lock);
	return 0;
}

SHELL_SUBCMD_ADD((kernel), ps, NULL, "Show thread CPU and stack usage", cmd_ps,
		 1, 0);

/* Time for the shell to send the command's last line before a restart. */
#define RESTART_DELAY_MS 100

/* Restarts into the boot ROM's USB loader (the RPI-RP2 drive), as holding
 * BOOTSEL at reset does: the ROM's reset_usb_boot() (hal_rpi_pico's
 * pico_bootrom, which Zephyr's SoC code calls for its retention boot mode),
 * with both of its interfaces and no activity LED, as the FreeRTOS build's
 * "bootsel".
 */
static int cmd_bootsel(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "Entering BOOTSEL (the RPI-RP2 drive)");
	k_msleep(RESTART_DELAY_MS);
	reset_usb_boot(0, 0);
}

SHELL_CMD_REGISTER(bootsel, NULL, "Restart into the USB bootloader (BOOTSEL)", cmd_bootsel);

/* The kernel has run the application's SYS_INIT() parts and started its
 * K_THREAD_DEFINE() threads before this (common/imu/app_main.h).
 */
int main(void)
{
	/* "nina passthrough confirm" asked for the NINA passthrough mode
	 * (common/drivers/nina/include/nina_passthrough.h): it runs instead of all that
	 * follows (no settings, no Wi-Fi, no time synchronization), until it
	 * restarts the board into the normal firmware.
	 */
#ifdef CONFIG_NINA_PASSTHROUGH
	if (nina_passthrough_active()) {
		nina_passthrough_run();
	}
#endif

	app_main();
	return 0;
}
