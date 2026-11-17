/* SPDX-License-Identifier: Apache-2.0 */
/* Subcommand dispatch with the Zephyr shell's output (see subcmd.h). */

#include <ctype.h>
#include <fnmatch.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "subcmd.h"

/* CONFIG_SHELL_DEFAULT_TERMINAL_WIDTH of the Zephyr build. */

#define TERMINAL_WIDTH 80

static void margin(size_t n)
{
  printf("%*s", (int)n, "");
}

/* formatted_text_print() of zephyr/subsys/shell/shell_help.c (without the
 * first-line margin, which no caller asks for): the text after its leading
 * white space, starting a new line at each '\n' and, where a line would
 * pass the terminal width, at the last white space before it (a word longer
 * than the line is cut at the width), each new line starting at the column
 * OFFSET, then a final newline.  The Zephyr shell moves the cursor to the
 * column with a VT100 sequence; this prints spaces.
 */

static void text_print(const char *str, size_t offset_col)
{
  size_t offset = 0;
  size_t length;

  if (str == NULL)
    {
      return;
    }

  while (isspace((unsigned char)str[offset]))
    {
      ++offset;
    }

  while (true)
    {
      size_t idx = 0;
      bool newline_found = false;

      length = strlen(str) - offset;
      if (length <= TERMINAL_WIDTH - offset_col)
        {
          for (idx = 0; idx < length; idx++)
            {
              if (str[offset + idx] == '\n')
                {
                  fwrite(str + offset, 1, idx, stdout);
                  offset += idx + 1;
                  putchar('\n');
                  margin(offset_col);
                  newline_found = true;
                  break;
                }
            }

          if (newline_found)
            {
              continue;
            }

          fputs(str + offset, stdout);
          break;
        }

      length = TERMINAL_WIDTH - offset_col;
      while (true)
        {
          if (isspace((unsigned char)str[offset + idx]))
            {
              length = idx;
              if (str[offset + idx] == '\n')
                {
                  break;
                }
            }

          if (idx + offset_col >= TERMINAL_WIDTH)
            {
              break;
            }

          ++idx;
        }

      fwrite(str + offset, 1, length, stdout);
      offset += length;
      while (isspace((unsigned char)str[offset]))
        {
          bool is_newline = str[offset] == '\n';

          ++offset;
          if (is_newline)
            {
              break;
            }
        }

      putchar('\n');
      margin(offset_col);
    }

  putchar('\n');
}

void subcmd_text_print(const char *str, size_t offset_col)
{
  text_print(str, offset_col);
}

/* z_shell_help_cmd_print() and z_shell_help_subcmd_print() with
 * help_item_print()'s layout: two spaces, the name padded to the longest
 * name of the set, two spaces, ": ", the help; the help text laid out by
 * text_print() from the column after "<name> - " or after ": ".
 */

static void help_print(const struct subcmd *entry)
{
  const struct subcmd *sub;
  int longest = 0;

  printf("%s - ", entry->syntax);
  text_print(entry->help, strlen(entry->syntax) + 3);
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
          printf("  %-*s  : ", longest, sub->syntax);
          text_print(sub->help, 2 * 2 + longest + 2);
        }
      else
        {
          printf("  %s\n", sub->syntax);
        }
    }
}

void subcmd_help(const struct subcmd *entry)
{
  help_print(entry);
  fflush(stdout);
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
