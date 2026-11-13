/* SPDX-License-Identifier: Apache-2.0 */
/* Subcommand dispatch with the Zephyr shell's output (see subcmd.h). */

#include <fnmatch.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "subcmd.h"

/* z_shell_help_cmd_print() and z_shell_help_subcmd_print() with
 * help_item_print()'s layout: two spaces, the name padded to the longest
 * name of the set, two spaces, ": ", the help.
 */

static void help_print(const struct subcmd *entry)
{
  const struct subcmd *sub;
  int longest = 0;

  printf("%s - %s\n", entry->syntax, entry->help ? entry->help : "");
  if (entry->subcmds == NULL)
    {
      return;
    }

  for (sub = entry->subcmds; sub->syntax != NULL; sub++)
    {
      int n = (int)strlen(sub->syntax);

      longest = n > longest ? n : longest;
    }

  if (longest == 0)
    {
      return;
    }

  printf("Subcommands:\n");
  for (sub = entry->subcmds; sub->syntax != NULL; sub++)
    {
      if (sub->help != NULL)
        {
          printf("  %-*s  : %s\n", longest, sub->syntax, sub->help);
        }
      else
        {
          printf("  %s\n", sub->syntax);
        }
    }
}

static const struct subcmd *find(const struct subcmd *parent,
                                 const char *word)
{
  const struct subcmd *sub;

  if (parent->subcmds == NULL)
    {
      return NULL;
    }

  for (sub = parent->subcmds; sub->syntax != NULL; sub++)
    {
      if (strcmp(sub->syntax, word) == 0)
        {
          return sub;
        }
    }

  return NULL;
}

static bool has_wildcard(const char *word)
{
  return strchr(word, '*') != NULL || strchr(word, '?') != NULL;
}

/* A wildcard word that matches none of PARENT's subcommands
 * (commands_expand() returns SHELL_WILDCARD_CMD_NO_MATCH_FOUND).
 */

static bool wildcard_no_match(const struct subcmd *parent, const char *word)
{
  const struct subcmd *sub;

  if (!has_wildcard(word))
    {
      return false;
    }

  if (parent->subcmds != NULL)
    {
      for (sub = parent->subcmds; sub->syntax != NULL; sub++)
        {
          if (fnmatch(word, sub->syntax, 0) == 0)
            {
              return false;
            }
        }
    }

  return true;
}

static int finish(int ret)
{
  fflush(stdout);
  return ret == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

int subcmd_main(const struct subcmd *root, int argc, char **argv)
{
  const struct subcmd *entry = root;      /* deepest matched entry */
  const struct subcmd *active = NULL;     /* deepest one with a handler */
  int active_lvl = 0;                     /* its index in argv */
  int lvl;

  if (root->handler != NULL)
    {
      active = root;
    }

  /* Walk down the sets while the words name subcommands. */

  for (lvl = 1; lvl < argc; lvl++)
    {
      const struct subcmd *sub;

      if (wildcard_no_match(entry, argv[lvl]))
        {
          /* The shell stops reading the words here. */

          argc = lvl;
          break;
        }

      sub = find(entry, argv[lvl]);
      if (sub == NULL)
        {
          break;
        }

      entry = sub;
      if (sub->handler != NULL)
        {
          active = sub;
          active_lvl = lvl;
        }
    }

  if (active == NULL)
    {
      help_print(entry);
      return finish(1);
    }

  argc -= active_lvl;
  argv += active_lvl;
  if (active->mandatory != 0)
    {
      int mand = active->mandatory;
      int opt = active->optional == SUBCMD_OPT_ANY ? 0xffff :
                active->optional;

      if (argc < mand || argc > mand + opt)
        {
          fprintf(stderr, "%s: wrong parameter count\n", active->syntax);
          help_print(active);
          return finish(1);
        }
    }

  return finish(active->handler(argc, argv));
}
