/* SPDX-License-Identifier: MIT */
/* "kv wipe confirm" (kv_store_os.h): erase the key-value area and start the
 * settings again on the empty area.
 *
 * The copy of Zephyr's settings (common/compat/subsys/settings) keeps its
 * state in settings_subsys_initialized (settings_init.c) and serializes its
 * calls with settings_lock (settings.c, settings_lock_take() of
 * settings_priv.h).  The wipe holds that lock from the erase until NVS is
 * mounted on the empty area, so no call of the settings, the key-value
 * store or the Wi-Fi credentials sees the area half erased or an NVS that
 * still points into the erased sectors.  Zephyr's k_mutex is recursive (so
 * is the layer's), which lets settings_subsys_init() take the lock again.
 */

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include "kv_store_os.h"
#include "flash_kv.h"
#include "settings_priv.h"

/* settings_init.c */
extern bool settings_subsys_initialized;

int kv_store_wipe(void)
{
	int ret, init;

	settings_lock_take();
	ret = flash_kv_wipe();
	/* Also after a failed erase: NVS must not keep its addresses into a
	 * sector that may have been erased.
	 */
	settings_subsys_initialized = false;
	init = settings_subsys_init();
	settings_lock_release();
	return ret ? ret : init;
}

int kv_store_cmd_wipe(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	if (strcmp(argv[1], "confirm") != 0) {
		shell_error(sh, "Type \"kv wipe confirm\" to erase all keys and Wi-Fi credentials");
		return -EINVAL;
	}
	ret = kv_store_wipe();
	if (ret) {
		shell_error(sh, "KV wipe failed: %d", ret);
	} else {
		shell_print(sh, "Erased");
	}
	return ret;
}
