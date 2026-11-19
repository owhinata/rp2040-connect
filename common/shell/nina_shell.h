/* SPDX-License-Identifier: MIT */
/* The "nina" command's subcommands (nina_shell.c) with a build's own
 * subcommand merged in: the FreeRTOS build adds "passthrough"
 * (freertos/drivers/nina/nina_passthrough.c) to the shared static set as a
 * dynamic set; its host test is freertos/tests/passthrough.
 */

#ifndef NINA_SHELL_H_
#define NINA_SHELL_H_

#include <stddef.h>
#include <string.h>

#include <zephyr/shell/shell.h>

/* The IDX-th of the static set SHARED (ending with an entry without a
 * syntax) with EXTRA put in by name, in ENTRY (no syntax past the end),
 * for a dynamic set (Zephyr sorts a command's subcommands by name).
 */
static inline void nina_pt_subcmd_get(const struct shell_static_entry *shared,
				      const struct shell_static_entry *extra, size_t idx,
				      struct shell_static_entry *entry)
{
	size_t before = 0; /* the shared entries before EXTRA */
	size_t n = 0;

	while (shared[n].syntax != NULL) {
		if (strcmp(shared[n].syntax, extra->syntax) < 0) {
			before = n + 1;
		}
		n++;
	}
	memset(entry, 0, sizeof(*entry));
	if (idx < before) {
		*entry = shared[idx];
	} else if (idx == before) {
		*entry = *extra;
	} else if (idx <= n) {
		*entry = shared[idx - 1];
	}
}

#endif /* NINA_SHELL_H_ */
