#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log_msg.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/shell/shell.h>

#define LOG_RAM_SIZE 4096
#define LOG_RAM_MAGIC 0x4c4f4752U

struct retained_log {
	uint32_t magic;
	uint32_t magic_inverse;
	size_t start;
	size_t length;
	char buffer[LOG_RAM_SIZE];
};

/* The linker leaves this region untouched during normal startup. */
static __noinit struct retained_log retained;
/* Bytes pushed out of the buffer since boot. Protected by log_lock. */
static uint64_t dropped;
static struct k_spinlock log_lock;
static uint8_t format_buffer[128];

static int store_output(uint8_t *data, size_t length, void *ctx)
{
	k_spinlock_key_t key;

	ARG_UNUSED(ctx);
	key = k_spin_lock(&log_lock);
	for (size_t i = 0; i < length; i++) {
		if (retained.length == LOG_RAM_SIZE) {
			retained.start = (retained.start + 1) % LOG_RAM_SIZE;
			retained.length--;
			dropped++;
		}
		retained.buffer[(retained.start + retained.length) % LOG_RAM_SIZE] = data[i];
		retained.length++;
	}
	k_spin_unlock(&log_lock, key);
	return (int)length;
}

LOG_OUTPUT_DEFINE(ram_output, store_output, format_buffer, sizeof(format_buffer));

static void ram_process(const struct log_backend *const backend, union log_msg_generic *msg)
{
	ARG_UNUSED(backend);
	log_output_msg_process(&ram_output, &msg->log,
			       LOG_OUTPUT_FLAG_TIMESTAMP | LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP |
			       LOG_OUTPUT_FLAG_LEVEL | LOG_OUTPUT_FLAG_CRLF_LFONLY);
}

static void ram_panic(const struct log_backend *const backend)
{
	ARG_UNUSED(backend);
}

static log_timestamp_t timestamp_us(void)
{
	return k_cyc_to_us_floor64(k_cycle_get_64());
}

static void ram_init(const struct log_backend *const backend)
{
	ARG_UNUSED(backend);
	log_set_timestamp_func(timestamp_us, 1000000U);
	if (retained.magic != LOG_RAM_MAGIC || retained.magic_inverse != ~LOG_RAM_MAGIC ||
	    retained.start >= LOG_RAM_SIZE || retained.length > LOG_RAM_SIZE) {
		retained.start = 0;
		retained.length = 0;
		retained.magic_inverse = ~LOG_RAM_MAGIC;
		retained.magic = LOG_RAM_MAGIC;
	}
}

static const struct log_backend_api ram_backend_api = {
	.process = ram_process,
	.panic = ram_panic,
	.init = ram_init,
};

LOG_BACKEND_DEFINE(ram_backend, ram_backend_api, true);

/* Prints the log in pieces so that no second copy of it is needed. `dropped`
 * numbers the bytes from boot, which keeps the position valid when new
 * messages push old ones out while printing. */
static int cmd_dmesg(const struct shell *sh, size_t argc, char **argv)
{
	char piece[97];
	uint64_t position = 0;
	bool first = true, any = false;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	/* Bounded in case logging keeps up with printing. */
	for (size_t printed = 0; printed < 2U * LOG_RAM_SIZE;) {
		k_spinlock_key_t key = k_spin_lock(&log_lock);
		size_t available, n;

		if (first || position < dropped) {
			position = dropped; /* start, or skip what was overwritten */
			first = false;
		}
		available = (size_t)(dropped + retained.length - position);
		n = MIN(available, sizeof(piece) - 1);
		for (size_t k = 0; k < n; k++) {
			piece[k] = retained.buffer[(retained.start + (size_t)(position - dropped) + k) %
						   LOG_RAM_SIZE];
		}
		k_spin_unlock(&log_lock, key);
		if (!n) {
			break;
		}
		piece[n] = '\0';
		shell_fprintf(sh, SHELL_NORMAL, "%s", piece);
		position += n;
		printed += n;
		any = true;
	}
	if (!any) {
		shell_print(sh, "No logs.");
	}
	return 0;
}

SHELL_CMD_REGISTER(dmesg, NULL, "Show logs stored in RAM", cmd_dmesg);
