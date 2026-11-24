/* SPDX-License-Identifier: MIT */
/* Host test of the "kv" command of the NuttX build: z_shell_main_kv() of
 * common/imu/kv_store.c (the Zephyr build's command, compiled with
 * Z_SHELL_CMD_NAMED_MAIN as apps/imu builds it; apps/kv's main() calls
 * it) and "kv wipe" (common/compat/kv_store_wipe.c), on
 * common/shell/zshell.c with the NuttX backends (common/shell/port/nuttx/
 * zshell_os_impl.h) and the key-value store on a fake flash (the copies of
 * Zephyr's settings and NVS, common/compat/zephyr_flash_kv.c,
 * common/tests/kv_flash_fake.c).
 *
 * The expected output is the NuttX build's before the command was shared
 * (apps/kv/kv_main.c on subcmd_main(): the same text, errors on stderr),
 * which is the Zephyr build's text: "Saved", "Deleted", "Reserved key",
 * the hex value, the error lines, the help and the argument errors of the
 * Zephyr shell; the help now lists "wipe", which only these builds have.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include <zephyr/settings/settings.h>

#include "kv_flash_fake.h"
#include "kv_store.h"

int z_shell_main_kv(int argc, char *argv[]);

/* The log (app_log.h of tests/lsm6dso_fakes). */

static char log_buf[4096];

void test_log(int level, const char *module, const char *fmt, ...)
{
	static const char *const names[] = {"", "err", "wrn", "inf", "dbg"};
	size_t n = strlen(log_buf);
	va_list ap;

	n += snprintf(log_buf + n, sizeof(log_buf) - n, "<%s> %s: ", names[level], module);
	va_start(ap, fmt);
	n += vsnprintf(log_buf + n, sizeof(log_buf) - n, fmt, ap);
	va_end(ap);
	snprintf(log_buf + n, sizeof(log_buf) - n, "\n");
}

/* Running a command line with stdout and stderr captured. */

static char dir[] = "/tmp/kvXXXXXX";
static char out_path[32], err_path[32];
static char out[8192], err[4096];

static void slurp(const char *path, char *buf, size_t size)
{
	FILE *f = fopen(path, "r");
	size_t n;

	assert(f);
	n = fread(buf, 1, size - 1, f);
	assert(n < size - 1);
	buf[n] = '\0';
	fclose(f);
}

static int run(const char *first, ...)
{
	char words[8][300];
	char *argv[9];
	int argc = 0, ret, saved_out, saved_err, fd;
	const char *w;
	va_list ap;

	va_start(ap, first);
	for (w = first; w; w = va_arg(ap, const char *)) {
		snprintf(words[argc], sizeof(words[argc]), "%s", w);
		argv[argc] = words[argc];
		argc++;
	}
	va_end(ap);
	argv[argc] = NULL;
	log_buf[0] = '\0';
	fflush(stdout);
	fflush(stderr);
	saved_out = dup(1);
	saved_err = dup(2);
	fd = open(out_path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
	assert(fd >= 0);
	dup2(fd, 1);
	close(fd);
	fd = open(err_path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
	assert(fd >= 0);
	dup2(fd, 2);
	close(fd);
	ret = z_shell_main_kv(argc, argv);
	fflush(stdout);
	fflush(stderr);
	dup2(saved_out, 1);
	dup2(saved_err, 2);
	close(saved_out);
	close(saved_err);
	slurp(out_path, out, sizeof(out));
	slurp(err_path, err, sizeof(err));
	return ret;
}

#define OUT(text) assert(strcmp(out, (text)) == 0 && err[0] == '\0')
#define ERR(text) assert(out[0] == '\0' && strcmp(err, (text)) == 0)

static const char kv_help[] =
	"kv - Application key-value storage\n"
	"Subcommands:\n"
	"  set     : Store a text value: <key> <value>\n"
	"  sethex  : Store hex bytes: <key> <hex>\n"
	"  get     : Print a value as hex: <key>\n"
	"  delete  : Delete a value: <key>\n"
	"  wipe    : Erase the whole store (also Wi-Fi credentials): confirm\n";

int main(void)
{
	char hex[300], text[200], key[40];
	uint8_t buf[KV_STORE_VALUE_MAX];

	assert(mkdtemp(dir));
	snprintf(out_path, sizeof(out_path), "%s/o", dir);
	snprintf(err_path, sizeof(err_path), "%s/e", dir);
	setvbuf(stdout, NULL, _IOLBF, 64);

	/* Data that is not Zephyr's NVS (a child process: the check of the
	 * area runs once per boot): every command fails with -EILSEQ and
	 * writes nothing, until "kv wipe confirm".
	 */
	kv_fake_init();
	memset(kv_fake->flash, 0x5a, sizeof(kv_fake->flash));
	{
		pid_t pid = fork();
		int status;

		assert(pid >= 0);
		if (pid == 0) {
			snprintf(hex, sizeof(hex), "KV read failed: %d\n", -EILSEQ);
			assert(run("kv", "get", "imu.port", NULL) == EXIT_FAILURE);
			ERR(hex);
			snprintf(hex, sizeof(hex), "KV write failed: %d\n", -EILSEQ);
			assert(run("kv", "set", "imu.port", "1", NULL) == EXIT_FAILURE);
			ERR(hex);
			snprintf(hex, sizeof(hex), "KV delete failed: %d\n", -EILSEQ);
			assert(run("kv", "delete", "imu.port", NULL) == EXIT_FAILURE);
			ERR(hex);
			assert(kv_fake->programs == 0 && kv_fake->erases == 0);
			assert(run("kv", "wipe", "no", NULL) == EXIT_FAILURE);
			assert(kv_fake->erases == 0 && kv_fake->flash[100] == 0x5a);
			assert(run("kv", "wipe", "confirm", NULL) == EXIT_SUCCESS);
			OUT("Erased\n");
			assert(run("kv", "set", "imu.port", "18830", NULL) == EXIT_SUCCESS);
			OUT("Saved\n");
			fflush(stdout);
			_exit(0);
		}
		assert(waitpid(pid, &status, 0) == pid && WIFEXITED(status) &&
		       WEXITSTATUS(status) == 0);
	}
	kv_fake_init();

	/* Help and argument counts (the Zephyr shell's). */
	assert(run("kv", NULL) == EXIT_FAILURE);
	OUT(kv_help);
	assert(run("kv", "foo", NULL) == EXIT_FAILURE);
	OUT(kv_help);
	assert(run("kv", "set", "k", NULL) == EXIT_FAILURE);
	assert(!strcmp(err, "set: wrong parameter count\n") &&
	       !strcmp(out, "set - Store a text value: <key> <value>\n"));
	assert(run("kv", "get", NULL) == EXIT_FAILURE);
	assert(!strcmp(err, "get: wrong parameter count\n") &&
	       !strcmp(out, "get - Print a value as hex: <key>\n"));
	assert(run("kv", "delete", "a", "b", NULL) == EXIT_FAILURE);
	assert(!strcmp(err, "delete: wrong parameter count\n"));
	assert(run("kv", "wipe", NULL) == EXIT_FAILURE);
	assert(!strcmp(err, "wipe: wrong parameter count\n") &&
	       !strcmp(out, "wipe - Erase the whole store (also Wi-Fi credentials): confirm\n"));
	assert(kv_fake->programs == 0 && kv_fake->erases == 0);

	/* set, get, sethex, delete. */
	assert(run("kv", "set", "imu.port", "18830", NULL) == EXIT_SUCCESS);
	OUT("Saved\n");
	assert(kv_store_get("imu.port", buf, sizeof(buf)) == 5 && !memcmp(buf, "18830", 5));
	assert(run("kv", "get", "imu.port", NULL) == EXIT_SUCCESS);
	OUT("3138383330\n");
	assert(run("kv", "sethex", "k.hex", "00ff7fA5aB", NULL) == EXIT_SUCCESS);
	OUT("Saved\n");
	assert(run("kv", "get", "k.hex", NULL) == EXIT_SUCCESS);
	OUT("00ff7fa5ab\n");
	assert(run("kv", "delete", "k.hex", NULL) == EXIT_SUCCESS);
	OUT("Deleted\n");
	assert(run("kv", "get", "k.hex", NULL) == EXIT_FAILURE);
	ERR("KV read failed: -2\n");
	assert(run("kv", "delete", "k.hex", NULL) == EXIT_SUCCESS);
	OUT("Deleted\n");

	/* The limits: 128 bytes, 32 characters. */
	memset(hex, 'A', 256);
	hex[256] = '\0';
	assert(run("kv", "sethex", "big", hex, NULL) == EXIT_SUCCESS);
	OUT("Saved\n");
	assert(run("kv", "get", "big", NULL) == EXIT_SUCCESS);
	memset(hex, 'a', 256);
	hex[256] = '\n';
	hex[257] = '\0';
	OUT(hex);
	memset(hex, '0', 258);
	hex[258] = '\0';
	assert(run("kv", "sethex", "big", hex, NULL) == EXIT_FAILURE);
	ERR("Expected 1-128 bytes as hexadecimal pairs\n");
	assert(run("kv", "sethex", "big", "abc", NULL) == EXIT_FAILURE);
	ERR("Expected 1-128 bytes as hexadecimal pairs\n");
	assert(run("kv", "sethex", "big", "0g", NULL) == EXIT_FAILURE);
	ERR("Invalid hexadecimal value\n");
	memset(text, 't', 128);
	text[128] = '\0';
	assert(run("kv", "set", "big", text, NULL) == EXIT_SUCCESS);
	OUT("Saved\n");
	text[128] = 't';
	text[129] = '\0';
	assert(run("kv", "set", "big", text, NULL) == EXIT_FAILURE);
	ERR("KV write failed: -22\n");
	memset(key, 'k', 32);
	key[32] = '\0';
	assert(run("kv", "set", key, "v", NULL) == EXIT_SUCCESS);
	OUT("Saved\n");
	key[32] = 'k';
	key[33] = '\0';
	assert(run("kv", "set", key, "v", NULL) == EXIT_FAILURE);
	ERR("KV write failed: -22\n");
	assert(run("kv", "get", key, NULL) == EXIT_FAILURE);
	ERR("KV read failed: -22\n");
	assert(run("kv", "delete", "a/b", NULL) == EXIT_FAILURE);
	ERR("KV delete failed: -22\n");

	/* "sys." is the shell's no-go. */
	assert(kv_store_set("sys.wifi_profile", "p", 1) == 0);
	assert(run("kv", "set", "sys.x", "1", NULL) == EXIT_FAILURE);
	ERR("Reserved key\n");
	assert(run("kv", "sethex", "sys.x", "01", NULL) == EXIT_FAILURE);
	ERR("Reserved key\n");
	assert(run("kv", "get", "sys.wifi_profile", NULL) == EXIT_FAILURE);
	ERR("Reserved key\n");
	assert(run("kv", "delete", "sys.wifi_profile", NULL) == EXIT_FAILURE);
	ERR("Reserved key\n");
	assert(kv_store_get("sys.wifi_profile", buf, sizeof(buf)) == 1);
	assert(run("kv", "set", "sys", "1", NULL) == EXIT_SUCCESS);
	OUT("Saved\n");
	assert(run("kv", "set", "SYS.x", "1", NULL) == EXIT_SUCCESS);
	OUT("Saved\n");

	/* wipe needs the word "confirm". */
	assert(run("kv", "wipe", "yes", NULL) == EXIT_FAILURE);
	ERR("Type \"kv wipe confirm\" to erase all keys and Wi-Fi credentials\n");
	assert(run("kv", "wipe", "CONFIRM", NULL) == EXIT_FAILURE);
	assert(kv_store_get("imu.port", buf, sizeof(buf)) == 5 && kv_fake->erases == 0);
	assert(settings_save_one("wifi_cred/0", "c", 1) == 0);
	assert(run("kv", "wipe", "confirm", NULL) == EXIT_SUCCESS);
	OUT("Erased\n");
	assert(kv_fake->erases == 2);
	assert(run("kv", "get", "imu.port", NULL) == EXIT_FAILURE);
	ERR("KV read failed: -2\n");
	assert(settings_load_one("wifi_cred/0", buf, sizeof(buf)) == 0);
	assert(run("kv", "set", "imu.port", "18830", NULL) == EXIT_SUCCESS);
	OUT("Saved\n");

	unlink(out_path);
	unlink(err_path);
	rmdir(dir);
	puts("kv_cmd ok");
	return 0;
}
