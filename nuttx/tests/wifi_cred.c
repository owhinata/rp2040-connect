/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of the stored Wi-Fi credentials and the "wifi cred" command
 * (nuttx/apps/wifi): the copies of Zephyr's wifi_credentials.c, its
 * settings backend on the layer's settings (here a fake store of named
 * items standing in for /dev/config) and wifi_credentials_shell.c, on
 * ../subcmd and the layer's Wi-Fi helpers (zephyr_wifi_utils.c).  The
 * expected output is what Zephyr's sources print (the same text; see
 * zephyr/zephyr/subsys/net/lib/wifi_credentials/); the stored entry is
 * struct wifi_credentials_personal under "wifi_cred/<index>".
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <assert.h>
#include <errno.h>
#include <getopt.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_credentials.h>
#include <zephyr/settings/settings.h>

int wifi_main(int argc, char *argv[]);

/* The log (app_log.h of tests/lsm6dso_fakes). */

static char log_buf[2048];

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

/* The settings: a few named items. */

#define ITEMS 8

static struct item {
	char name[64];
	unsigned char value[512];
	size_t len;
	bool used;
} items[ITEMS];
static int store_error;     /* returned by the next call when set */
static int saves, deletes;

static struct item *find(const char *name)
{
	for (int i = 0; i < ITEMS; i++) {
		if (items[i].used && !strcmp(items[i].name, name)) {
			return &items[i];
		}
	}
	return NULL;
}

static int take_error(void)
{
	int ret = store_error;

	store_error = 0;
	return ret;
}

int settings_subsys_init(void)
{
	return take_error();
}

int settings_save_one(const char *name, const void *value, size_t val_len)
{
	struct item *it = find(name);
	int ret = take_error();

	if (ret) {
		return ret;
	}
	if (!val_len) {
		return settings_delete(name);
	}
	saves++;
	if (!it) {
		for (int i = 0; i < ITEMS && !it; i++) {
			if (!items[i].used) {
				it = &items[i];
			}
		}
		assert(it);
		snprintf(it->name, sizeof(it->name), "%s", name);
		it->used = true;
	}
	assert(val_len <= sizeof(it->value));
	memcpy(it->value, value, val_len);
	it->len = val_len;
	return 0;
}

int settings_delete(const char *name)
{
	struct item *it = find(name);
	int ret = take_error();

	if (ret) {
		return ret;
	}
	deletes++;
	if (it) {
		it->used = false;
	}
	return 0;
}

ssize_t settings_load_one(const char *name, void *buf, size_t buf_len)
{
	struct item *it = find(name);
	int ret = take_error();

	if (ret) {
		return ret;
	}
	if (!it) {
		return 0;
	}
	memcpy(buf, it->value, it->len < buf_len ? it->len : buf_len);
	return it->len;
}

/* The interface and NET_REQUEST_WIFI_CONNECT_STORED. */

static struct net_if iface;
static int connect_stored_calls, connect_stored_ret;

struct net_if *net_if_get_wifi_sta(void)
{
	return &iface;
}

int net_mgmt_NET_REQUEST_WIFI_CONNECT_STORED(uint64_t mgmt_request, struct net_if *ifc,
					     void *data, size_t len)
{
	assert(mgmt_request == NET_REQUEST_WIFI_CONNECT_STORED);
	assert(ifc == &iface && data == NULL && len == 0);
	connect_stored_calls++;
	return connect_stored_ret;
}

/* Running the command with stdout and stderr captured. */

static char out[8192], err[2048];

static void slurp(FILE *f, char *buf, size_t size)
{
	size_t n;

	fflush(f);
	rewind(f);
	n = fread(buf, 1, size - 1, f);
	buf[n] = '\0';
	/* An embedded NUL ("%c" of 0) shows as '@'. */
	for (size_t i = 0; i < n; i++) {
		if (buf[i] == '\0') {
			buf[i] = '@';
		}
	}
}

static int run(const char *first, ...)
{
	char words[16][80];
	char *argv[17];
	int argc = 0, ret, saved_out, saved_err;
	FILE *fo = tmpfile(), *fe = tmpfile();
	const char *w;
	va_list ap;

	assert(fo && fe);
	va_start(ap, first);
	for (w = first; w; w = va_arg(ap, const char *)) {
		snprintf(words[argc], sizeof(words[argc]), "%s", w);
		argv[argc] = words[argc];
		argc++;
	}
	va_end(ap);
	argv[argc] = NULL;
	/* A fresh getopt() state, as a new NuttX task has. */
	optind = 0;
	opterr = 0;
	log_buf[0] = '\0';
	fflush(stdout);
	fflush(stderr);
	saved_out = dup(1);
	saved_err = dup(2);
	dup2(fileno(fo), 1);
	dup2(fileno(fe), 2);
	ret = wifi_main(argc, argv);
	fflush(stdout);
	fflush(stderr);
	dup2(saved_out, 1);
	dup2(saved_err, 2);
	close(saved_out);
	close(saved_err);
	slurp(fo, out, sizeof(out));
	slurp(fe, err, sizeof(err));
	fclose(fo);
	fclose(fe);
	return ret;
}

#define OK(...)   assert(run("wifi", "cred", __VA_ARGS__, NULL) == EXIT_SUCCESS)
#define FAIL(...) assert(run("wifi", "cred", __VA_ARGS__, NULL) == EXIT_FAILURE)

static const struct wifi_credentials_personal *entry(int idx)
{
	char name[16];
	struct item *it;

	snprintf(name, sizeof(name), "wifi_cred/%d", idx);
	it = find(name);
	if (!it) {
		return NULL;
	}
	assert(it->len == sizeof(struct wifi_credentials_personal));
	return (const struct wifi_credentials_personal *)it->value;
}

static const char add_help_start[] =
	"add - Add network to storage.\n"
	"      <-s --ssid \"<SSID>\">: SSID.\n"
	"      [-c --channel]: Channel that needs to be scanned for connection. 0:any\n"
	"      channel.\n";

int main(void)
{
	struct wifi_credentials_personal pre = {0};
	char s32[33], s33[34], p64[65], p65[66];

	/* An entry stored before boot is in the cache after the init. */
	pre.header.type = WIFI_SECURITY_TYPE_PSK;
	memcpy(pre.header.ssid, "dummy-pre", 9);
	pre.header.ssid_len = 9;
	memcpy(pre.password, "dummypass0", 10);
	pre.password_len = 10;
	assert(settings_save_one("wifi_cred/1", &pre, sizeof(pre)) == 0);
	assert(wifi_credentials_is_empty());
	assert(wifi_credentials_init() == 0 && log_buf[0] == '\0');
	assert(!wifi_credentials_is_empty());
	OK("list");
	assert(!strcmp(out, "  network ssid: \"dummy-pre\", ssid_len: 9, type: WPA2-PSK, "
			    "password: \"dummypass0\", password_len: 10, MFP_OPTIONAL\n"));
	assert(err[0] == '\0');

	/* A stored entry of the wrong length: the load fails. */
	find("wifi_cred/1")->len = sizeof(pre) - 1;
	OK("list");
	assert(out[0] == '\0');
	assert(!strcmp(err, "An error occurred when trying to load credentials for network "
			    "\"dummy-pre\". err: -22\n"));
	assert(!strcmp(log_buf, "<err> wifi_credentials_backend: Settings error: invalid "
				"settings length\n<err> wifi_credentials: Failed to load WiFi "
				"credentials at index 1, err: -22\n"));
	find("wifi_cred/1")->len = sizeof(pre);

	/* Delete. */
	FAIL("delete");
	assert(!strcmp(out, "Usage: wifi cred delete \"network name\"\n") && err[0] == '\0');
	FAIL("delete", "nosuch");
	assert(!strcmp(out, "\tDeleting network ssid: \"nosuch\", ssid_len: 6\n"));
	assert(!strcmp(err, "No matching network with SSID: \"nosuch\" found\n"));
	FAIL("delete", "0123456789012345678901234567890123");
	assert(!strcmp(err, "SSID too long\n") && out[0] == '\0');
	OK("delete", "dummy-pre");
	assert(!strcmp(out, "\tDeleting network ssid: \"dummy-pre\", ssid_len: 9\n"));
	assert(!find("wifi_cred/1") && wifi_credentials_is_empty());
	OK("list");
	assert(out[0] == '\0' && err[0] == '\0');

	/* Add: no output on success; the entry as stored. */
	OK("add", "-s", "dummy-a", "-k", "1", "-p", "dummypass1");
	assert(out[0] == '\0' && err[0] == '\0');
	assert(entry(0) && entry(0)->header.type == WIFI_SECURITY_TYPE_PSK);
	assert(entry(0)->header.ssid_len == 7 && !memcmp(entry(0)->header.ssid, "dummy-a", 7));
	assert(entry(0)->password_len == 10 && !memcmp(entry(0)->password, "dummypass1", 10));
	assert(entry(0)->header.flags == 0 && entry(0)->header.channel == 0);
	/* Every stored name is outside the kv command's "app_kv/". */
	for (int i = 0; i < ITEMS; i++) {
		assert(!items[i].used || !strncmp(items[i].name, "wifi_cred/", 10));
	}

	/* Lengths: SSID 1-32, passphrase 8-64. */
	memset(s32, 's', 32);
	s32[32] = '\0';
	memcpy(s33, s32, 32);
	strcpy(s33 + 32, "x");
	memset(p64, 'p', 64);
	p64[64] = '\0';
	memcpy(p65, p64, 64);
	strcpy(p65 + 64, "q");
	FAIL("add", "-s", s33, "-k", "1", "-p", "dummypass1");
	assert(!strcmp(err, "SSID too long (max 32 characters)\n\n") && out[0] == '\0');
	FAIL("add", "-s", "dummy-b", "-k", "1", "-p", "dummyp7");
	assert(!strcmp(err, "Passphrase should be minimum 8 characters\n\n"));
	FAIL("add", "-s", "dummy-b", "-k", "1", "-p", p65);
	assert(!strcmp(err, "Passphrase too long (max 64 characters)\n\n"));
	FAIL("add", "-k", "1", "-p", "dummypass1");
	assert(!strcmp(err, "SSID not provided\n\n"));
	assert(!strncmp(out, add_help_start, strlen(add_help_start)));
	FAIL("add", "-s", "", "-k", "1");
	assert(!strcmp(err, "SSID not provided\n\n"));
	OK("add", "-s", s32, "-k", "1", "-p", p64);
	assert(out[0] == '\0' && err[0] == '\0');
	assert(entry(1)->header.ssid_len == 32 && entry(1)->password_len == 64);

	/* No room for a third; the log says so. */
	FAIL("add", "-s", "dummy-c", "-k", "1", "-p", "dummypass3");
	assert(!strcmp(err, "Failed to add network credentials, err: -105\n"));
	assert(!strcmp(log_buf, "<err> wifi_credentials: Cannot store WiFi credentials, no "
				"space left\n"));

	/* The same SSID replaces its entry in place, with the new options. */
	OK("add", "-s", "dummy-a", "-k", "1", "-p", "dummypass4", "-c", "6", "-w", "2");
	OK("add", "-s", s32, "-k", "0", "-p", "dummypass5");
	assert(!strcmp(err, "Passphrase provided without security configuration\n\n"));
	OK("list");
	assert(!strcmp(out,
		"  network ssid: \"dummy-a\", ssid_len: 7, type: WPA2-PSK, password: "
		"\"dummypass4\", password_len: 10, channel: 6, MFP_REQUIRED\n"
		"  network ssid: \"ssssssssssssssssssssssssssssssss\", ssid_len: 32, type: "
		"OPEN, MFP_OPTIONAL\n"));
	OK("add", "-s", "dummy-a", "-k", "3", "-p", "dummypass6", "-b", "2", "-m",
	   "02:00:00:00:00:0a", "-w", "0");
	OK("add", "--ssid=dummy-a", "--key-mgmt", "1", "--passphrase", "dummypass7", "--band",
	   "5", "--channel", "36");
	OK("delete", s32);
	OK("add", "-s", "dummy b", "-k", "3", "-p", "dummypass8", "-b", "2", "-m",
	   "02:00:00:00:00:0a", "-w", "0");
	OK("list");
	assert(!strcmp(out,
		"  network ssid: \"dummy-a\", ssid_len: 7, type: WPA2-PSK, password: "
		"\"dummypass7\", password_len: 10, band: 5GHz, channel: 36, MFP_OPTIONAL\n"
		"  network ssid: \"dummy b\", ssid_len: 7, type: WPA3-SAE-HNP, password: "
		"\"dummypass8\", password_len: 10, bssid: 02:00:00:00:00:0a, band: 2.4GHz, "
		"MFP_DISABLED\n"));
	/* OWE drops the passphrase. */
	OK("add", "-s", "dummy b", "-k", "23", "-p", "dummypass9");
	assert(!strcmp(err, "Passphrase is not required for OWE connection\n\n"));
	assert(entry(1)->password_len == 0 && entry(1)->password[0] == '\0');

	/* Option errors. */
	FAIL("add", "-s", "dummy-a", "-x");
	assert(!strcmp(err, "Invalid option x\n\n"));
	FAIL("add", "-s", "dummy-a", "-t", "5");
	assert(!strcmp(err, "Invalid option @\n\n"));
	FAIL("add", "-k", "1", "-s");
	assert(!strcmp(err, "Invalid option s\n\n"));
	FAIL("add", "-s", "dummy-a", "-c", "15");
	assert(!strcmp(err, "Invalid channel: 15, checked bands: 2.4GHz,5GHz,6GHz\n\n"));
	FAIL("add", "-s", "dummy-a", "-c", "6x");
	assert(!strcmp(err, "Invalid channel: 6x\n\n"));
	FAIL("add", "-s", "dummy-a", "-b", "3");
	assert(!strcmp(err, "Invalid band: 3\n\n"));
	FAIL("add", "-s", "dummy-a", "-w", "2");
	assert(!strcmp(err, "MFP not supported for security type OPEN\n"));
	FAIL("add", "-s", "dummy-a", "-k", "1", "-w", "3");
	assert(!strcmp(err, "Invalid IEEE 802.11w value: 3\n"));
	FAIL("add", "-s", "dummy-a", "-m", "zz");
	assert(!strcmp(err, "Invalid MAC address\n\n"));
	FAIL("add", "-h");
	assert(!strncmp(out, add_help_start, strlen(add_help_start)) && err[0] == '\0');
	FAIL("add");
	assert(!strcmp(err, "add: wrong parameter count\n"));
	assert(!strncmp(out, add_help_start, strlen(add_help_start)));
	/* A store error. */
	store_error = -ENOSPC;
	FAIL("add", "-s", "dummy-a", "-k", "1", "-p", "dummypass1");
	assert(!strcmp(err, "Failed to add network credentials, err: -28\n"));
	assert(!strcmp(log_buf, "<err> wifi_credentials: Failed to store WiFi credentials at "
				"index 0, err: -28\n"));

	/* auto_connect: the request, an error printed, the result always 0. */
	OK("auto_connect");
	assert(connect_stored_calls == 1 && out[0] == '\0' && err[0] == '\0');
	connect_stored_ret = -EINVAL;
	OK("auto_connect");
	assert(!strcmp(err, "An error occurred when trying to auto-connect to a network. "
			    "err: -22\n"));

	/* The help of "wifi" and "wifi cred" (subcmd's layout of Zephyr's). */
	assert(run("wifi", NULL) == EXIT_FAILURE);
	assert(!strcmp(out, "wifi - Wi-Fi commands\nSubcommands:\n"
			    "  cred  : Wifi credentials management.\n          \n"));
	assert(run("wifi", "cred", NULL) == EXIT_FAILURE);
	assert(!strncmp(out, "cred - Wifi credentials management.\n       \nSubcommands:\n"
			     "  add           : Add network to storage.\n", 98));
	assert(strstr(out, "\n  delete        : Delete network from storage.\n"
			   "                  \n  list          : List stored networks.\n"
			   "                  \n  auto_connect  : Connect to any stored network.\n"
			   "                  \n"));

	/* Delete everything: empty again. */
	OK("delete", "dummy-a");
	OK("delete", "dummy b");
	assert(wifi_credentials_is_empty() && !entry(0) && !entry(1));
	OK("list");
	assert(out[0] == '\0');

	puts("wifi_cred tests passed");
	return 0;
}
