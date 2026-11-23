/* SPDX-License-Identifier: MIT */
/* Host test of the "i2c" command (common/shell/i2c_shell.c, the
 * copy of the Zephyr build's I2C shell, as nuttx/apps/i2c builds it) on the
 * Zephyr shell's command handling of common/shell/zshell.c and the
 * compatibility layer's I2C bus device (compat/zephyr_i2c.c, the devices of
 * common/compat/zephyr_device.c), against a fake NuttX I2C master that
 * records each transfer (frequency, flags, lengths), answers WHO_AM_I
 * (0x6c at 0x0f) and 0xa0, 0xa1, ... for other reads, refuses a message of
 * no bytes (-ENOTSUP, as the board's driver) and its I2C_RESET()
 * (-ENOTSUP).  The expected text is the Zephyr build's, from the copied
 * code and zephyr/subsys/shell:
 *
 * - the devices of the arguments: the ready I2C devices (i2c@40044000, not
 *   the LSM6DSOX's), in the help that a wrong argument count prints; no
 *   such device.
 * - "i2c speed": the bus device's frequency for the later transfers, which
 *   i2c_get_config() reports; a class the controller does not have is
 *   refused and changes nothing.
 * - "i2c read_byte" (write NOSTOP + read), "i2c write_byte" (one write),
 *   "i2c read" in Zephyr's hexdump, "i2c direct_read" (one read).
 * - "i2c scan": the master refuses the write of no bytes, so the scan stops
 *   at its first address with patch 0004's message; "i2c recover":
 *   I2C_RESET()'s -ENOTSUP.
 * - i2c_transfer(): STOP on the last message, NuttX's NOSTOP and NOSTART
 *   from Zephyr's flags, the address and the bus's frequency on each.
 */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/i2c/i2c_master.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

int i2c_main(int argc, char *argv[]);

/* The fake master -------------------------------------------------------*/

static char bus_log[1024];
static uint8_t reg;

static void note(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

static void note(const char *fmt, ...)
{
	size_t used = strlen(bus_log);
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(bus_log + used, sizeof(bus_log) - used, fmt, ap);
	va_end(ap);
}

static void expect_bus(const char *want)
{
	if (strcmp(bus_log, want) != 0) {
		fprintf(stderr, "--- bus got:\n%s\n--- want:\n%s\n", bus_log, want);
		assert(0);
	}
	bus_log[0] = '\0';
}

static int fake_transfer(struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count)
{
	(void)dev;
	for (int i = 0; i < count; i++) {
		if (msgs[i].length == 0) {
			return -ENOTSUP;
		}
	}
	for (int i = 0; i < count; i++) {
		note("%u:%x:%x:%d ", (unsigned int)msgs[i].frequency, msgs[i].addr, msgs[i].flags,
		     (int)msgs[i].length);
		for (ssize_t j = 0; j < msgs[i].length; j++) {
			if (msgs[i].flags & I2C_M_READ) {
				msgs[i].buffer[j] = reg == 0x0f ? 0x6c : (uint8_t)(0xa0 + j);
			} else if (j == 0) {
				reg = msgs[i].buffer[0];
			}
		}
	}
	return 0;
}

static int fake_reset(struct i2c_master_s *dev)
{
	(void)dev;
	return -ENOTSUP;
}

static const struct i2c_ops_s fake_ops = {.transfer = fake_transfer, .reset = fake_reset};
static struct i2c_master_s fake_master = {.ops = &fake_ops};

/* The devices of zephyr_device.c's list: the bus (as the board's
 * rp2040_lsm6dso.c defines it) and the LSM6DSOX's, which is not an I2C
 * device.
 */
static struct i2c_compat_bus bus;

DEVICE_DT_DEFINE(i2c0, "i2c@40044000", NULL, &bus, NULL, NULL);
DEVICE_DT_DEFINE(lsm6dsox, "lsm6dsox@6a", NULL, NULL, NULL, NULL);

/* The session's terminal (as tests/sensor_shell.c) ----------------------*/

static char term_path[] = "/tmp/zsh_XXXXXX";
static int term_fd = -1;
static char text[16384];

static const char *term_text(void)
{
	ssize_t n = pread(term_fd, text, sizeof(text) - 1, 0);

	assert(n >= 0);
	text[n] = '\0';
	return text;
}

static int run_v(int argc, char **args)
{
	char *argv[16];
	int saved_out, saved_err, ret;

	argv[0] = "i2c";
	for (int i = 1; i < argc; i++) {
		argv[i] = args[i - 1];
	}
	argv[argc] = NULL;

	assert(ftruncate(term_fd, 0) == 0);
	fflush(stdout);
	fflush(stderr);
	saved_out = dup(1);
	saved_err = dup(2);
	dup2(term_fd, 1);
	dup2(term_fd, 2);
	ret = i2c_main(argc, argv);
	fflush(stdout);
	fflush(stderr);
	dup2(saved_out, 1);
	dup2(saved_err, 2);
	close(saved_out);
	close(saved_err);
	term_text();
	return ret;
}

#define run(...)                                                                                   \
	run_v(1 + (int)(sizeof((char *[]){__VA_ARGS__}) / sizeof(char *)), (char *[]){__VA_ARGS__})

static void expect(const char *want)
{
	if (strcmp(text, want) != 0) {
		fprintf(stderr, "--- got:\n%s--- want:\n%s---\n", text, want);
		assert(0);
	}
}

/* The tests --------------------------------------------------------------*/

static const struct device *const i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));

static void test_devices(void)
{
	DEVICE_DT_GET(DT_NODELABEL(lsm6dsox))->state->initialized = true;
	assert(run("read_byte", "i2c@40044000") == EXIT_FAILURE);
	assert(strstr(text, "read_byte: wrong parameter count\n") == text);
	assert(strstr(text, "Usage: read_byte <device> <addr> <reg>\n") != NULL);
	assert(strstr(text, "Subcommands:\n  i2c@40044000\n") != NULL);
	assert(strstr(text, "lsm6dsox") == NULL);
	DEVICE_DT_GET(DT_NODELABEL(lsm6dsox))->state->initialized = false;

	assert(run("read_byte", "i2c9", "6a", "0f") == EXIT_FAILURE);
	expect("I2C: Device driver i2c9 not found.\n");
	expect_bus("");
}

static void test_speed(void)
{
	uint32_t config = 0;

	/* At the frequency of the board's start, 1 MHz. */
	assert(i2c_get_config(i2c0, &config) == 0);
	assert(config == (I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_FAST_PLUS)));
	assert(run("read_byte", "i2c@40044000", "6a", "0f") == EXIT_SUCCESS);
	expect("Output: 0x6c\n");
	expect_bus("1000000:6a:40:1 1000000:6a:1:1 ");

	assert(run("speed", "i2c@40044000", "1") == EXIT_SUCCESS);
	expect("");
	assert(i2c_compat_frequency(i2c0) == 100000);
	assert(i2c_get_config(i2c0, &config) == 0);
	assert(config == (I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD)));
	assert(run("read_byte", "i2c@40044000", "6a", "0f") == EXIT_SUCCESS);
	expect("Output: 0x6c\n");
	expect_bus("100000:6a:40:1 100000:6a:1:1 ");

	assert(run("speed", "i2c@40044000", "2") == EXIT_SUCCESS);
	assert(i2c_compat_frequency(i2c0) == 400000);

	/* Refused: nothing changes. */
	assert(run("speed", "i2c@40044000", "4") == EXIT_FAILURE);
	expect("I2C: Failed to configure device: i2c@40044000\n");
	assert(run("speed", "i2c@40044000", "0") == EXIT_FAILURE);
	expect("I2C: Failed to configure device: i2c@40044000\n");
	assert(i2c_compat_frequency(i2c0) == 400000);

	assert(run("speed", "i2c@40044000", "3") == EXIT_SUCCESS);
	assert(i2c_compat_frequency(i2c0) == 1000000);
	expect_bus("");
}

static void test_read_write(void)
{
	assert(run("write_byte", "i2c@40044000", "6a", "12", "44") == EXIT_SUCCESS);
	expect("");
	expect_bus("1000000:6a:0:2 ");

	assert(run("read", "i2c@40044000", "6a", "78", "14") == EXIT_SUCCESS);
	expect("00000000: a0 a1 a2 a3 a4 a5 a6 a7  a8 a9 aa ab ac ad ae af |........ ........|\n"
	       "00000010: b0 b1 b2 b3                                      |....             |\n");
	expect_bus("1000000:6a:40:1 1000000:6a:1:20 ");

	assert(run("direct_read", "i2c@40044000", "6a", "4") == EXIT_SUCCESS);
	expect("00000000: a0 a1 a2 a3                                      |....             |\n");
	expect_bus("1000000:6a:1:4 ");
}

static void test_scan_recover(void)
{
	char want[512];
	int n;

	assert(run("scan", "i2c@40044000") == EXIT_FAILURE);
	n = snprintf(want, sizeof(want),
		     "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n00: ");
	for (int i = 0; i < 4; i++) {
		n += snprintf(want + n, sizeof(want) - n, "   ");
	}
	snprintf(want + n, sizeof(want) - n,
		 "\nI2C: i2c@40044000 does not support this scan (zero-length writes)\n");
	expect(want);
	expect_bus("");

	assert(run("recover", "i2c@40044000") == EXIT_FAILURE);
	snprintf(want, sizeof(want), "I2C: Bus recovery failed (err %d)\n", -ENOTSUP);
	expect(want);
}

static void test_transfer(void)
{
	uint8_t a[2] = {0x10, 0x11}, b = 0x12, c[3];
	struct i2c_msg msgs[3] = {
		{.buf = a, .len = 2, .flags = I2C_MSG_WRITE},
		{.buf = &b, .len = 1, .flags = I2C_MSG_WRITE},
		{.buf = c, .len = 3, .flags = I2C_MSG_RESTART | I2C_MSG_READ},
	};
	struct i2c_msg five[5] = {{0}};

	/* The second write goes on from the first (NOSTART), the read starts
	 * again; STOP on the last only.
	 */
	assert(i2c_transfer(i2c0, msgs, 3, 0x6a) == 0);
	assert(msgs[2].flags & I2C_MSG_STOP);
	expect_bus("1000000:6a:40:2 1000000:6a:c0:1 1000000:6a:1:3 ");

	/* A STOP in between: a new START, no NOSTART. */
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	msgs[1].flags = I2C_MSG_READ;
	assert(i2c_transfer(i2c0, msgs, 2, 0x6a) == 0);
	expect_bus("1000000:6a:0:2 1000000:6a:1:1 ");

	assert(i2c_transfer(i2c0, msgs, 0, 0x6a) == 0);
	assert(i2c_transfer(i2c0, five, 5, 0x6a) == -EINVAL);
	expect_bus("");
}

int main(void)
{
	term_fd = mkstemp(term_path);
	assert(term_fd >= 0);
	close(term_fd);
	term_fd = open(term_path, O_RDWR | O_APPEND);
	assert(term_fd >= 0);

	assert(!device_is_ready(i2c0));
	assert(run("read_byte", "i2c@40044000", "6a", "0f") == EXIT_FAILURE);
	expect("I2C: Device driver i2c@40044000 not found.\n");
	assert(i2c_compat_bind(i2c0, &fake_master, 1000000) == 0);

	test_devices();
	test_speed();
	test_read_write();
	test_scan_recover();
	test_transfer();

	close(term_fd);
	unlink(term_path);
	return 0;
}
