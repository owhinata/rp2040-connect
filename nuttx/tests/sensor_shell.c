/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of the "sensor" command (nuttx/apps/sensor: the copies of the
 * Zephyr build's sensor_shell.c and Zephyr's sensor_shell_stream.c) on the
 * Zephyr shell's command handling of nuttx/apps/subcmd/zshell.c, the ported
 * LSM6DSO driver and its fake sensor (tests/lsm6dso_driver.c, included
 * below; its own tests run first).  The expected text is the Zephyr
 * build's, derived from the copied code and zephyr/subsys/shell
 * (shell.c execute(), shell_help.c, shell_wildcard.c):
 *
 * - help: "sensor" alone, a subcommand that the build leaves out ("info",
 *   CONFIG_SENSOR_INFO), the structured help (SHELL_HELP(): description,
 *   "Usage: <name> <usage>") wrapped at 80 columns, the dynamic device set
 *   ("Subcommands:" and the device name), "wrong parameter count".
 * - the devices: node name and node label for get, attr_*, trig; the node
 *   name only for stream (device_get_binding()), unknown names.
 * - "sensor get": all channels and named ones, an unknown channel, the
 *   line format of a 3-axis channel (type, index, shift, num_samples,
 *   value in ns and Q31 as PRIq(6)), printed by the processing callback
 *   after the read's completion on the terminal of the command (the
 *   backend's path), a wildcard that names channels and one that names
 *   none (the shell stops reading the words there).
 * - attr_set / attr_get: the FIFO attributes, the value parser (micro-units,
 *   leading zeros, signs, invalid text), unknown channel or attribute, the
 *   driver's errors, "%.6f" values, -EBUSY while a stream runs.
 * - stream: the argument errors, "Enabling stream...", a batch printed by
 *   the processing thread's callback with its trigger line, "off" and the
 *   cancellation that prints nothing.
 * - trig: the driver has no trigger_set (-ENOSYS) and the slot stays taken,
 *   as in the copied code.
 */
#define main lsm6dso_driver_tests
#include "lsm6dso_driver.c"
#undef main

#include <fcntl.h>
#include <unistd.h>

#include <zephyr/shell/shell.h>

#include "sensor_shell.h"
#include "sensor_shell_nuttx.h"

int sensor_main(int argc, char *argv[]);

/* The session's terminal: a file that fd 1 and 2 point at while a command
 * runs, opened for appending (as the backend opens it).
 */
static char term_path[] = "/tmp/zsh_XXXXXX";
static int term_fd = -1;
static char text[262144];

static void term_clear(void)
{
	assert(ftruncate(term_fd, 0) == 0);
}

static const char *term_text(void)
{
	ssize_t n = pread(term_fd, text, sizeof(text) - 1, 0);

	assert(n >= 0);
	text[n] = '\0';
	return text;
}

/* One turn of the processing thread (sensor_processing_with_callback()
 * with the thread's callback) if a completion is there; the fake kernel
 * cannot wait.
 */
static bool process_one(void)
{
	struct rtio_cqe *cqe = rtio_cqe_consume(&sensor_read_rtio);
	uint8_t *buf = NULL;
	uint32_t buf_len = 0;
	void *userdata;
	int rc;

	if (cqe == NULL) {
		return false;
	}
	rc = cqe->result;
	userdata = cqe->userdata;
	rtio_cqe_get_mempool_buffer(&sensor_read_rtio, cqe, &buf, &buf_len);
	rtio_cqe_release(&sensor_read_rtio, cqe);
	sensor_shell_nuttx_processing_callback(rc, buf, buf_len, userdata);
	rtio_release_buffer(&sensor_read_rtio, buf, buf_len);
	return true;
}

/* Lets the work queues run and the processing thread take what has
 * completed; the terminal's text after it.
 */
static const char *drain(void)
{
	do {
		test_work_run_all();
	} while (process_one());
	return term_text();
}

/* Runs "sensor ARGS..." with stdout and stderr on the terminal, then
 * drain(); text is what the terminal shows.
 */
static int run_v(int argc, char **args)
{
	char *argv[40];
	int saved_out, saved_err, ret;

	argv[0] = "sensor";
	for (int i = 1; i < argc; i++) {
		argv[i] = args[i - 1];
	}
	argv[argc] = NULL;

	term_clear();
	fflush(stdout);
	fflush(stderr);
	saved_out = dup(1);
	saved_err = dup(2);
	dup2(term_fd, 1);
	dup2(term_fd, 2);
	ret = sensor_main(argc, argv);
	fflush(stdout);
	fflush(stderr);
	dup2(saved_out, 1);
	dup2(saved_err, 2);
	close(saved_out);
	close(saved_err);
	drain();
	return ret;
}

#define run(...) \
	run_v(1 + (int)(sizeof((char *[]){__VA_ARGS__}) / sizeof(char *)), (char *[]){__VA_ARGS__})

static int run0(void)
{
	return run_v(1, NULL);
}

static void expect(const char *want)
{
	if (strcmp(text, want) != 0) {
		fprintf(stderr, "--- got:\n%s--- want:\n%s---\n", text, want);
		assert(0);
	}
}

static int lines(void)
{
	int n = 0;

	for (const char *p = text; *p; p++) {
		n += *p == '\n';
	}
	return n;
}

static const char sensor_help[] =
	"sensor - Sensor commands\n"
	"Subcommands:\n"
	"  get       : Get sensor data.\n"
	"              Channel names are optional. All channels are read when no channels\n"
	"              are provided.\n"
	"              Usage: get <device_name> [<channel name 0> .. <channel name N>]\n"
	"  attr_set  : Set the sensor's channel attribute.\n"
	"              Usage: attr_set <device_name> <channel_name> <attribute_name> <value>\n"
	"  attr_get  : Get the sensor's channel attribute.\n"
	"              Usage: attr_get <device_name> [<channel_name 0> <attribute_name 0> ..\n"
	"                     <channel_name N> <attribute_name N>]\n"
	"  stream    : Start/stop streaming sensor data.\n"
	"              Data ready trigger will be used if no triggers are provided.\n"
	"              Usage: stream <device_name> on|off <trigger name> incl|drop|nop\n"
	"  trig      : Get or set the trigger type on a sensor.\n"
	"              Currently only supports `data_ready`.\n"
	"              Usage: trig <device_name> <on/off> <trigger_name>\n";

static void test_help(void)
{
	/* No handler: the help of the deepest entry with one. */
	assert(run0() == EXIT_FAILURE);
	expect(sensor_help);
	/* "info" is left out (CONFIG_SENSOR_INFO): an unknown word, the root's
	 * help; also "-h" (CONFIG_SHELL_HELP_OPT_PARSE is off).
	 */
	assert(run("info") == EXIT_FAILURE);
	expect(sensor_help);
	assert(run("-h") == EXIT_FAILURE);
	expect(sensor_help);
	/* A wildcard that matches subcommands is replaced by them; with no
	 * handler on the way, the root's help.  One followed by a command with
	 * a handler is refused.
	 */
	assert(run("g*") == EXIT_FAILURE);
	expect(sensor_help);
	assert(run("g*", "attr_set") == EXIT_FAILURE);
	expect("Error: requested multiple function executions\n");

	/* mandatory 2: the error, the entry's help, its dynamic device set. */
	assert(run("get") == EXIT_FAILURE);
	expect("get: wrong parameter count\n"
	       "get - Get sensor data.\n"
	       "      Channel names are optional. All channels are read when no channels are\n"
	       "      provided.\n"
	       "      Usage: get <device_name> [<channel name 0> .. <channel name N>]\n"
	       "Subcommands:\n"
	       "  lsm6dsox@6a\n");
	assert(run("attr_get") == EXIT_FAILURE);
	expect("attr_get: wrong parameter count\n"
	       "attr_get - Get the sensor's channel attribute.\n"
	       "           Usage: attr_get <device_name> [<channel_name 0> <attribute_name 0> ..\n"
	       "                  <channel_name N> <attribute_name N>]\n"
	       "Subcommands:\n"
	       "  lsm6dsox@6a\n");
	assert(run("attr_set") == EXIT_FAILURE);
	expect("attr_set: wrong parameter count\n"
	       "attr_set - Set the sensor's channel attribute.\n"
	       "           Usage: attr_set <device_name> <channel_name> <attribute_name> <value>\n"
	       "Subcommands:\n"
	       "  lsm6dsox@6a\n");
	assert(run("trig") == EXIT_FAILURE);
	expect("trig: wrong parameter count\n"
	       "trig - Get or set the trigger type on a sensor.\n"
	       "       Currently only supports `data_ready`.\n"
	       "       Usage: trig <device_name> <on/off> <trigger_name>\n"
	       "Subcommands:\n"
	       "  lsm6dsox@6a\n");
}

static void test_devices(void)
{
	/* get, attr_*, trig: the node name or the node label of a sensor. */
	assert(run("get", "nosuch") == EXIT_FAILURE);
	expect("Sensor device unknown (nosuch)\n");
	assert(run("get", "i2c0") == EXIT_FAILURE);
	expect("Sensor device unknown (i2c0)\n");
	assert(run("attr_get", "i2c@40044000") == EXIT_FAILURE);
	expect("Sensor device unknown (i2c@40044000)\n");
	assert(run("attr_set", "nosuch", "all", "fifo_watermark", "1") == EXIT_FAILURE);
	expect("Sensor device unknown (nosuch)\n");
	assert(run("trig", "nosuch", "on", "data_ready") == EXIT_FAILURE);
	expect("Sensor device unknown (nosuch)\n");
	/* stream: the node name only (device_get_binding()). */
	assert(run("stream", "lsm6dsox", "off") == EXIT_FAILURE);
	expect("Device unknown (lsm6dsox)\n");
}

/* A line "channel type=T(NAME) index=0 shift=S num_samples=1 value=<ns>ns<tail>". */
static bool has_channel(int type, const char *name, const char *tail)
{
	char head[64];
	const char *p = text;

	snprintf(head, sizeof(head), "channel type=%d(%s) index=0 shift=", type, name);
	while ((p = strstr(p, head)) != NULL) {
		const char *eol = strchr(p, '\n');
		const char *ns = strstr(p, "ns");
		size_t n = strlen(tail);

		if (eol && ns && ns < eol && strstr(p, " num_samples=1 value=") < ns &&
		    (size_t)(eol - (ns + 2)) == n && strncmp(ns + 2, tail, n) == 0) {
			return true;
		}
		p++;
	}
	return false;
}

static void test_get(void)
{
	/* The result is printed by the processing thread's callback on the
	 * terminal of the command (the backend), after the read completes.
	 */
	set_raw(LSM6DSO_OUTX_L_A, 0, 0, 4100);
	set_raw(LSM6DSO_OUTX_L_G, 0, 0, 0);
	assert(run("get", "lsm6dsox", "accel_xyz") == EXIT_SUCCESS);
	assert(lines() == 4);
	assert(has_channel(0, "accel_x", " (0.000000)"));
	assert(has_channel(1, "accel_y", " (0.000000)"));
	assert(has_channel(2, "accel_z", " (9.810572)"));
	assert(has_channel(3, "accel_xyz", ", (0.000000, 0.000000, 9.810572)"));
	assert(strncmp(text, "channel type=0(accel_x) index=0 shift=4 num_samples=1 value=", 60) == 0);

	/* All channels (the 3-axis ones are not asked for separately). */
	assert(run("get", "lsm6dsox@6a") == EXIT_SUCCESS);
	assert(lines() == 8);
	assert(has_channel(3, "accel_xyz", ", (0.000000, 0.000000, 9.810572)"));
	/* The gyro has the bias tracked by the driver's tests: not 0. */
	assert(strstr(text, "channel type=7(gyro_xyz) index=0 shift=4 num_samples=1 value=") &&
	       strstr(text, "ns, (-0.012217, 0.006108, -0.002444)\n"));

	/* Unknown channels are skipped; none left is an error. */
	assert(run("get", "lsm6dsox", "foo") == EXIT_FAILURE);
	expect("Failed to read channel (foo)\nNo channels to read, bailing\n");
	assert(run("get", "lsm6dsox", "foo", "gyro_x") == EXIT_SUCCESS);
	assert(strncmp(text, "Failed to read channel (foo)\n", 29) == 0 && lines() == 2);
	assert(strstr(text, "channel type=4(gyro_x) index=0 shift="));
	/* A channel number. */
	assert(run("get", "lsm6dsox", "2") == EXIT_SUCCESS);
	assert(lines() == 1 && has_channel(2, "accel_z", " (9.810572)"));

	/* Wildcards: after the node name (a dynamic entry), a pattern is
	 * replaced by the matching channel names; one that matches none ends
	 * the words (all channels are read).  After the node label (not an
	 * entry of the set) the pattern is an ordinary word.
	 */
	assert(run("get", "lsm6dsox@6a", "gyro_?") == EXIT_SUCCESS);
	assert(lines() == 4 && strstr(text, "channel type=6(gyro_z) index=0 shift=") &&
	       strstr(text, "channel type=7(gyro_xyz) index=0 shift="));
	assert(run("get", "lsm6dsox@6a", "xyz*", "gyro_x") == EXIT_SUCCESS);
	assert(lines() == 8);
	assert(run("get", "lsm6dsox", "gyro_?") == EXIT_FAILURE);
	expect("Failed to read channel (gyro_?)\nNo channels to read, bailing\n");
}

static void test_attrs(void)
{
	char want[256];

	assert(run("attr_get", "lsm6dsox", "all", "fifo_watermark", "all",
		   "fifo_timestamp_decimation") == EXIT_SUCCESS);
	expect("lsm6dsox@6a(channel=all, attr=fifo_watermark) value=32.000000 value=0.000000 "
	       "value=0.000000\n"
	       "lsm6dsox@6a(channel=all, attr=fifo_timestamp_decimation) value=8.000000 "
	       "value=0.000000 value=0.000000\n");
	/* Numbers for the channel and the attribute. */
	assert(SENSOR_CHAN_ALL == 78 && SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK == 19);
	assert(run("attr_get", "lsm6dsox", "78", "fifo_watermark") == EXIT_SUCCESS);
	expect("lsm6dsox@6a(channel=78, attr=fifo_watermark) value=32.000000 value=0.000000 "
	       "value=0.000000\n");
	assert(run("attr_get", "lsm6dsox", "all", "19") == EXIT_SUCCESS);
	expect("lsm6dsox@6a(channel=all, attr=19) value=32.000000 value=0.000000 "
	       "value=0.000000\n");
	/* Errors: the names, the driver's (other attributes are not read). */
	assert(run("attr_get", "lsm6dsox", "bogus", "fifo_watermark") == EXIT_SUCCESS);
	expect("Channel 'bogus' unknown\n");
	assert(run("attr_get", "lsm6dsox", "all", "bogus") == EXIT_SUCCESS);
	expect("Attribute 'bogus' unknown\n");
	assert(run("attr_get", "lsm6dsox", "accel_xyz", "sampling_frequency") == EXIT_SUCCESS);
	snprintf(want, sizeof(want),
		 "Failed to get channel(accel_xyz) attribute(sampling_frequency): %d\n", -ENOTSUP);
	expect(want);
	/* No pairs: every channel and attribute, -EINVAL left out. */
	assert(run("attr_get", "lsm6dsox") == EXIT_SUCCESS);
	assert(lines() == SENSOR_CHAN_COMMON_COUNT * SENSOR_ATTR_COMMON_COUNT);
	snprintf(want, sizeof(want),
		 "Failed to get channel(accel_x) attribute(sampling_frequency): %d\n", -ENOTSUP);
	assert(strncmp(text, want, strlen(want)) == 0);

	/* attr_set: the FIFO attributes. */
	assert(run("attr_set", "lsm6dsox", "all", "fifo_watermark", "64") == EXIT_SUCCESS);
	expect("lsm6dsox@6a channel=all, attr=fifo_watermark set to value=64\n");
	assert(run("attr_set", "lsm6dsox", "all", "fifo_watermark", "48.000000", "all",
		   "fifo_timestamp_decimation", "32") == EXIT_SUCCESS);
	expect("lsm6dsox@6a channel=all, attr=fifo_watermark set to value=48.000000\n"
	       "lsm6dsox@6a channel=all, attr=fifo_timestamp_decimation set to value=32\n");
	for (int i = 0; i < 3; i++) {
		static char *const ok[] = {"0", "1", "8"};

		assert(run("attr_set", "lsm6dsox", "all", "fifo_timestamp_decimation", ok[i]) ==
		       EXIT_SUCCESS);
	}
	assert(run("attr_set", "lsm6dsox", "all", "fifo_timestamp_decimation", "3") ==
	       EXIT_SUCCESS);
	snprintf(want, sizeof(want),
		 "Failed to set channel(all) attribute(fifo_timestamp_decimation): %d\n", -EINVAL);
	expect(want);
	assert(run("attr_set", "lsm6dsox", "all", "fifo_watermark", "0") == EXIT_SUCCESS);
	snprintf(want, sizeof(want), "Failed to set channel(all) attribute(fifo_watermark): %d\n",
		 -EINVAL);
	expect(want);
	assert(run("attr_get", "lsm6dsox", "all", "fifo_watermark", "all",
		   "fifo_timestamp_decimation") == EXIT_SUCCESS);
	expect("lsm6dsox@6a(channel=all, attr=fifo_watermark) value=48.000000 value=0.000000 "
	       "value=0.000000\n"
	       "lsm6dsox@6a(channel=all, attr=fifo_timestamp_decimation) value=8.000000 "
	       "value=0.000000 value=0.000000\n");
	/* The value parser: micro-units, nothing finer, no other text. */
	static char *const bad[] = {"abc", "1.", "1.x", "-", "1.0000001", "99999999999", "1e3"};
	for (size_t i = 0; i < ARRAY_SIZE(bad); i++) {
		assert(run("attr_set", "lsm6dsox", "all", "fifo_watermark", bad[i]) ==
		       EXIT_FAILURE);
		snprintf(want, sizeof(want), "Sensor value '%s' invalid\n", bad[i]);
		expect(want);
	}
	assert(run("attr_set", "lsm6dsox", "bogus", "fifo_watermark", "1") == EXIT_FAILURE);
	expect("Channel 'bogus' unknown\n");
	assert(run("attr_set", "lsm6dsox", "all", "bogus", "1") == EXIT_FAILURE);
	expect("Attribute 'bogus' unknown\n");
	/* The standard attributes go to the driver: the ODR. */
	assert(run("attr_set", "lsm6dsox", "accel_xyz", "sampling_frequency", "104", "gyro_xyz",
		   "sampling_frequency", "104.000000") == EXIT_SUCCESS);
	expect("lsm6dsox@6a channel=accel_xyz, attr=sampling_frequency set to value=104\n"
	       "lsm6dsox@6a channel=gyro_xyz, attr=sampling_frequency set to value=104.000000\n");
}

static void test_streaming(void)
{
	struct lsm6dso_data *d = imu()->data;
	char want[128];

	assert(run("stream", "lsm6dsox@6a") == EXIT_FAILURE);
	expect("Wrong number of arguments (2)\n");
	assert(run("stream", "lsm6dsox@6a", "on", "fifo_wm") == EXIT_FAILURE);
	expect("Wrong number of arguments (4)\n");
	assert(run("stream", "lsm6dsox@6a", "maybe") == EXIT_FAILURE);
	expect("Unknown streaming operation (maybe)\n");
	assert(run("stream", "lsm6dsox@6a", "on", "bad", "incl") == EXIT_FAILURE);
	expect("Invalid trigger (bad)\n");
	assert(run("stream", "lsm6dsox@6a", "on", "fifo_wm", "bad") == EXIT_FAILURE);
	expect("Unknown trigger op (bad)\n");
	/* "off" with no stream: nothing. */
	assert(run("stream", "lsm6dsox@6a", "off") == EXIT_SUCCESS);
	expect("");

	/* On: the FIFO at the set watermark (48) and ODR; a batch is printed by
	 * the processing thread with its trigger line, the average of the
	 * batch's samples.
	 */
	fifo_words = 0;
	assert(run("stream", "lsm6dsox@6a", "on", "fifo_wm", "incl") == EXIT_SUCCESS);
	expect("Enabling stream...\n");
	assert(d->stream_active && regs[USER][LSM6DSO_FIFO_CTRL1] == 48);
	term_clear();
	push_pairs(30);
	drain();
	/* The decoder's Q31 has shift 9; the accel x is the mean of 100..129. */
	assert(strncmp(text, "Trigger (10 / fifo_wm) detected\n"
			     "channel type=3(accel_xyz) index=0 shift=9 num_samples=30 value=", 95) == 0);
	assert(strstr(text, "ns, (0.273977, 0.000000, 9.810572)\n"
			    "channel type=7(gyro_xyz) index=0 shift=9 num_samples=30 value=") != NULL);
	assert(lines() == 3);
	/* While it runs: the attributes are refused. */
	assert(run("attr_set", "lsm6dsox", "all", "fifo_watermark", "32") == EXIT_SUCCESS);
	snprintf(want, sizeof(want), "Failed to set channel(all) attribute(fifo_watermark): %d\n",
		 -EBUSY);
	expect(want);
	/* Off: the stream ends with a cancellation, which prints nothing. */
	assert(run("stream", "lsm6dsox@6a", "off") == EXIT_SUCCESS);
	test_timeout_fire(&d->stream_work.timeout);
	drain();
	expect("Disabling existing stream\n");
	assert(!d->stream_active);
	fifo_words = 0;
	update_int1();
}

static void test_trig(void)
{
	char want[128];

	/* The driver has no trigger_set (CONFIG_LSM6DSO_TRIGGER_NONE); the slot
	 * stays taken, so the next "on" finds none (the copied code).
	 */
	assert(run("trig", "lsm6dsox", "on", "data_ready") == EXIT_FAILURE);
	snprintf(want, sizeof(want), "Error while setting trigger 1 on device lsm6dsox (%d)\n",
		 -ENOSYS);
	expect(want);
	assert(run("trig", "lsm6dsox", "on", "data_ready") == EXIT_FAILURE);
	snprintf(want, sizeof(want),
		 "Unable to support more simultaneous sensor trigger devices\n"
		 "Error while setting trigger 1 on device lsm6dsox (%d)\n", -ENOTSUP);
	expect(want);
	assert(run("trig", "lsm6dsox", "on", "tap") == EXIT_FAILURE);
	snprintf(want, sizeof(want), "Unsupported trigger type (tap)\n");
	expect(want);
	assert(run("trig", "lsm6dsox", "maybe", "data_ready") == EXIT_FAILURE);
	expect("Pass 'on' or 'off' to enable/disable trigger\n");
	assert(run("trig", "lsm6dsox", "on") == EXIT_FAILURE);
	expect("Wrong number of args\n");
}

int main(void)
{
	lsm6dso_driver_tests();

	term_fd = mkstemp(term_path);
	assert(term_fd >= 0);
	close(term_fd);
	term_fd = open(term_path, O_RDWR | O_APPEND);
	assert(term_fd >= 0);

	test_help();
	test_devices();
	test_get();
	test_attrs();
	test_streaming();
	test_trig();

	close(term_fd);
	unlink(term_path);
	return 0;
}
