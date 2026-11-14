/* SPDX-License-Identifier: Apache-2.0 */
/* The Zephyr shell's command execution, help and backends for commands
 * whose sets are written with the Zephyr macros of the compatibility
 * layer's <zephyr/shell/shell.h> (SHELL_CMD_REGISTER(), static and dynamic
 * subcommand sets, SHELL_HELP()), as configured in the Zephyr build
 * (CONFIG_SHELL_HELP=y, CONFIG_SHELL_HELP_ON_WRONG_ARGUMENT_COUNT=y,
 * CONFIG_SHELL_WILDCARD=y, CONFIG_SHELL_ARGC_MAX=30,
 * CONFIG_SHELL_CMD_BUFF_SIZE=512, CONFIG_SHELL_DEFAULT_TERMINAL_WIDTH=80;
 * CONFIG_SHELL_HELP_OPT_PARSE, aliases and select mode off).
 *
 * z_shell_cmd_main() follows zephyr/subsys/shell/shell.c execute(),
 * exec_cmd(), active_cmd_prepare(), cmd_precheck(), wildcard_check_report(),
 * shell_wildcard.c and shell_help.c line by line, on the words that nsh has
 * split (nsh has already removed the quotes; the line that the wildcard
 * expansion rewrites is the words joined by single spaces, which is what
 * z_shell_spaces_trim() leaves of Zephyr's command buffer).  The handler
 * gets argv as Zephyr's, NULL after the last word (Zephyr's array is
 * zeroed); when the matching stopped at a wildcard that matched nothing,
 * argv[argc] is that word and argv[argc + 1] the rest of the line, as there.
 * Help and the argument count error go to stdout and stderr of the command,
 * which are the session's terminal; the Zephyr shell moves the cursor with
 * VT100 sequences where this prints spaces.
 *
 * Backends: a struct shell per terminal (the path of the command's stdout:
 * /dev/ttyACM0 for USB, /dev/telnet0 for Telnet, found by F_GETPATH or by
 * the inode number among /dev), kept for the life of the
 * system as Zephyr's shell instances are.  Output to it from the command
 * itself goes to its stdout and stderr; output from any other task or
 * thread (after the command has returned) opens the terminal, writes the
 * text with one write() and closes it again, so that no descriptor of a
 * Telnet session is held between two lines: when the client goes, the
 * session's device goes with its last descriptor (the next client gets the
 * slot), the output is dropped while no client is connected (open() fails)
 * and reaches the next client, as Zephyr's Telnet backend does.
 */

#ifdef __NuttX__
#  include <nuttx/config.h>
#endif

#include <sys/stat.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <fnmatch.h>
#include <limits.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <zephyr/device.h>
#include <zephyr/shell/shell.h>

#include "subcmd.h"

/* CONFIG_SHELL_* of the Zephyr build. */

#define ARGC_MAX        30    /* CONFIG_SHELL_ARGC_MAX */
#define CMD_BUFF_SIZE   512   /* CONFIG_SHELL_CMD_BUFF_SIZE */

/* Backends: USB, Telnet and a few redirected outputs. */

#define BACKENDS        4

/* A line that fits is written with one write(). */

#define LINE_MAX_ONCE   256

#define SHELL_MSG_CMD_NOT_FOUND ": command not found"
#define SHELL_MSG_TOO_MANY_ARGS "Too many arguments in the command.\n"

enum shell_wildcard_status
{
  SHELL_WILDCARD_CMD_ADDED,
  SHELL_WILDCARD_CMD_MISSING_SPACE,
  SHELL_WILDCARD_CMD_NO_MATCH_FOUND,  /* no matching command */
  SHELL_WILDCARD_NOT_FOUND            /* wildcard character not found */
};

/* The state of one command line (Zephyr: the shell's context). */

struct run
{
  char temp_buff[CMD_BUFF_SIZE];      /* the line for the expansion */
  uint16_t cmd_tmp_buff_len;          /* with the EOS */
  char rest[CMD_BUFF_SIZE];           /* the rest of the line */
  char *argv[ARGC_MAX + 1 + 4];       /* zeroed, as Zephyr's */
  struct shell_static_entry active_cmd;
};

static struct shell g_backends[BACKENDS];
static pthread_mutex_t g_backends_lock = PTHREAD_MUTEX_INITIALIZER;

/* Backends ----------------------------------------------------------------*/

/* The path of the calling task's stdout, or a negative errno.  NuttX:
 * F_GETPATH; a Telnet session's device passes it on to its socket, which
 * answers "socket:[...]", so a path that is not the same inode is looked
 * up among the devices of /dev by the inode number of fstat().
 */

#ifdef F_GETPATH
static int dev_lookup(const struct stat *want, char *path, size_t size)
{
  struct dirent *de;
  DIR *dir = opendir("/dev");
  int ret = -ENOENT;

  if (dir == NULL)
    {
      return -errno;
    }

  while ((de = readdir(dir)) != NULL)
    {
      struct stat st;
      int n = snprintf(path, size, "/dev/%s", de->d_name);

      if (n < 0 || (size_t)n >= size)
        {
          continue;
        }

      if (stat(path, &st) == 0 && S_ISCHR(st.st_mode) &&
          st.st_ino == want->st_ino)
        {
          ret = 0;
          break;
        }
    }

  closedir(dir);
  return ret;
}
#endif

static int stdout_path(char *path, size_t size)
{
#ifdef F_GETPATH
  char buf[PATH_MAX];
  struct stat out;
  struct stat st;

  if (fstat(STDOUT_FILENO, &out) < 0)
    {
      return -errno;
    }

  if (fcntl(STDOUT_FILENO, F_GETPATH, buf) < 0 || buf[0] != '/' ||
      stat(buf, &st) < 0 || st.st_ino != out.st_ino)
    {
      return dev_lookup(&out, path, size);
    }
#else
  /* Host test: Linux. */

  char buf[PATH_MAX];
  ssize_t n = readlink("/proc/self/fd/1", buf, sizeof(buf) - 1);

  if (n < 0)
    {
      return -errno;
    }

  buf[n] = '\0';
#endif

  if (strlen(buf) >= size)
    {
      return -ENAMETOOLONG;
    }

  strcpy(path, buf);
  return 0;
}

/* The backend of the calling task's terminal; NULL (its stdio) when the
 * path is not known or the table is full.
 */

static struct shell *backend_get(void)
{
  char path[Z_SHELL_PATH_MAX];
  struct shell *sh = NULL;
  size_t i;

  if (stdout_path(path, sizeof(path)) < 0)
    {
      return NULL;
    }

  pthread_mutex_lock(&g_backends_lock);
  for (i = 0; i < BACKENDS; i++)
    {
      if (strcmp(g_backends[i].path, path) == 0)
        {
          sh = &g_backends[i];
          break;
        }
    }

  for (i = 0; sh == NULL && i < BACKENDS; i++)
    {
      if (g_backends[i].path[0] == '\0')
        {
          sh = &g_backends[i];
          strcpy(sh->path, path);
        }
    }

  pthread_mutex_unlock(&g_backends_lock);
  return sh;
}

static void write_all(int fd, const char *buf, size_t len)
{
  while (len > 0)
    {
      ssize_t n = write(fd, buf, len);

      if (n <= 0)
        {
          if (n < 0 && errno == EINTR)
            {
              continue;
            }

          return;
        }

      buf += n;
      len -= (size_t)n;
    }
}

void z_shell_backend_vfprintf(const struct shell *sh,
                              enum shell_vt100_color color,
                              const char *fmt, va_list args)
{
  char buf[LINE_MAX_ONCE];
  va_list copy;
  bool own;
  int fd;
  int n;

  own = sh->cmd_pid != 0 && sh->cmd_pid == getpid();
  if (own)
    {
      /* The command itself: its stdout or stderr, after what it has
       * buffered on stdout.
       */

      fflush(stdout);
      fd = color == SHELL_WARNING || color == SHELL_ERROR ?
           STDERR_FILENO : STDOUT_FILENO;
    }
  else
    {
      fd = open(sh->path, O_WRONLY | O_APPEND);
      if (fd < 0)
        {
          return;   /* no such terminal now: dropped */
        }
    }

  va_copy(copy, args);
  n = vsnprintf(buf, sizeof(buf), fmt, copy);
  va_end(copy);
  if (n >= 0 && (size_t)n < sizeof(buf))
    {
      write_all(fd, buf, (size_t)n);
    }
  else if (n >= 0)
    {
      vdprintf(fd, fmt, args);
    }

  if (!own)
    {
      close(fd);
    }
}

/* Devices (subsys/shell/shell_utils.c) ------------------------------------*/

const struct device *shell_device_filter(size_t idx,
                                         shell_device_filter_t filter)
{
  size_t match_idx = 0;
  size_t i;

  /* shell_device_internal(idx, NULL, filter, SHELL_DEVICE_STATUS_READY) */

  for (i = 0; i < z_device_count(); i++)
    {
      const struct device *dev = z_device_get(i);

      if (device_is_ready(dev) && dev->name != NULL &&
          strlen(dev->name) != 0 && (filter == NULL || filter(dev)))
        {
          if (match_idx == idx)
            {
              return dev;
            }

          ++match_idx;
        }
    }

  return NULL;
}

const struct device *shell_device_get_binding(const char *name)
{
  const struct device *dev = device_get_binding(name);

  if (dev == NULL)
    {
      dev = device_get_by_dt_nodelabel(name);
    }

  return dev;
}

/* Commands (subsys/shell/shell_utils.c) -----------------------------------*/

static size_t z_strlen(const char *str)
{
  return str == NULL ? 0U : strlen(str);
}

/* z_shell_cmd_get(): the IDX-th subcommand of PARENT (the root command for
 * NULL), a dynamic one in DLOC.
 */

static const struct shell_static_entry *
cmd_get(const struct shell_static_entry *root,
        const struct shell_static_entry *parent, size_t idx,
        struct shell_static_entry *dloc)
{
  const struct shell_static_entry *res = NULL;

  if (parent == NULL)
    {
      return idx == 0 ? root : NULL;
    }

  if (parent->subcmd != NULL)
    {
      if (parent->subcmd->is_dynamic)
        {
          parent->subcmd->dynamic_get(idx, dloc);
          if (dloc->syntax != NULL)
            {
              res = dloc;
            }
        }
      else
        {
          const struct shell_static_entry *entry_list = parent->subcmd->entry;

          if (entry_list[idx].syntax != NULL)
            {
              res = &entry_list[idx];
            }
        }
    }

  return res;
}

/* z_shell_find_cmd() */

static const struct shell_static_entry *
find_cmd(const struct shell_static_entry *root,
         const struct shell_static_entry *parent, const char *cmd_str,
         struct shell_static_entry *dloc)
{
  const struct shell_static_entry *entry;
  size_t idx = 0;

  while ((entry = cmd_get(root, parent, idx++, dloc)) != NULL)
    {
      if (strcmp(cmd_str, entry->syntax) == 0)
        {
          return entry;
        }
    }

  return NULL;
}

/* Help (subsys/shell/shell_help.c) ----------------------------------------*/

static void margin(size_t n)
{
  printf("%*s", (int)n, "");
}

/* formatted_structured_help_print() */

static void structured_help_print(const char *item_name,
                                  const char *item_help,
                                  size_t terminal_offset)
{
  const struct shell_cmd_help *structured =
    (const struct shell_cmd_help *)item_help;

  if (structured->description)
    {
      subcmd_text_print(structured->description, terminal_offset);
    }

  if (structured->usage)
    {
      margin(terminal_offset);
      printf("Usage: %s ", item_name);
      subcmd_text_print(structured->usage, terminal_offset + 7);
    }
}

/* help_item_print() (picolibc: "%s%-*s") */

static void help_item_print(const char *item_name, uint16_t item_name_width,
                            const char *item_help)
{
  const uint16_t offset = 2 * 2 + item_name_width + 2;

  if (z_strlen(item_name) == 0)
    {
      return;
    }

  printf("  %-*s", item_name_width, item_name);
  if (item_help == NULL)
    {
      putchar('\n');
      return;
    }

  printf("  : ");
  if (shell_help_is_structured(item_help))
    {
      structured_help_print(item_name, item_help, offset);
    }
  else
    {
      subcmd_text_print(item_help, offset);
    }
}

/* z_shell_help_subcmd_print() */

static void help_subcmd_print(const struct shell_static_entry *parent,
                              const char *description)
{
  const struct shell_static_entry *entry = NULL;
  struct shell_static_entry dloc;
  uint16_t longest = 0U;
  size_t idx = 0;

  while ((entry = cmd_get(NULL, parent, idx++, &dloc)) != NULL)
    {
      uint16_t len = (uint16_t)z_strlen(entry->syntax);

      longest = len > longest ? len : longest;
    }

  if (longest == 0)
    {
      return;
    }

  if (description != NULL)
    {
      fputs(description, stdout);
    }

  idx = 0;
  while ((entry = cmd_get(NULL, parent, idx++, &dloc)) != NULL)
    {
      help_item_print(entry->syntax, longest, entry->help);
    }
}

/* z_shell_help_cmd_print() */

static void help_cmd_print(const struct shell_static_entry *cmd)
{
  size_t field_width = z_strlen(cmd->syntax) + 3;

  printf("%s - ", cmd->syntax);
  if (shell_help_is_structured(cmd->help))
    {
      structured_help_print(cmd->syntax, cmd->help, field_width);
    }
  else
    {
      subcmd_text_print(cmd->help, field_width);
    }
}

/* shell_internal_help_print() */

static void internal_help_print(const struct shell_static_entry *active_cmd)
{
  help_cmd_print(active_cmd);
  help_subcmd_print(active_cmd, "Subcommands:\n");
  fflush(stdout);
}

/* Wildcards (subsys/shell/shell_wildcard.c, shell_utils.c) ----------------*/

static bool has_wildcard(const char *str)
{
  return strchr(str, '?') != NULL || strchr(str, '*') != NULL;
}

static enum shell_wildcard_status command_add(char *buff, uint16_t *buff_len,
                                              const char *cmd,
                                              const char *pattern)
{
  uint16_t cmd_len = (uint16_t)z_strlen(cmd);
  char *completion_addr;
  uint16_t shift;

  /* +1 for space */

  if ((*buff_len + cmd_len + 1) > CMD_BUFF_SIZE)
    {
      return SHELL_WILDCARD_CMD_MISSING_SPACE;
    }

  completion_addr = strstr(buff, pattern);
  if (!completion_addr)
    {
      return SHELL_WILDCARD_CMD_NO_MATCH_FOUND;
    }

  shift = (uint16_t)z_strlen(completion_addr);

  /* make place for new command: + 1 for space + 1 for EOS */

  memmove(completion_addr + cmd_len + 1, completion_addr, shift + 1);
  memcpy(completion_addr, cmd, cmd_len);

  /* adding space to not brake next command in the buffer */

  completion_addr[cmd_len] = ' ';
  *buff_len += cmd_len + 1; /* + 1 for space */
  return SHELL_WILDCARD_CMD_ADDED;
}

/* z_shell_pattern_remove() */

static void pattern_remove(char *buff, uint16_t *buff_len,
                           const char *pattern)
{
  char *pattern_addr = strstr(buff, pattern);
  uint16_t pattern_len = (uint16_t)z_strlen(pattern);
  uint16_t shift;

  if (!pattern_addr)
    {
      return;
    }

  if (pattern_addr > buff)
    {
      if (*(pattern_addr - 1) == ' ')
        {
          pattern_len++;  /* space needs to be removed as well */
          pattern_addr--; /* set pointer to space */
        }
    }

  shift = (uint16_t)(z_strlen(pattern_addr) - pattern_len + 1); /* EOS */
  *buff_len -= pattern_len;
  memmove(pattern_addr, pattern_addr + pattern_len, shift);
}

/* commands_expand() */

static enum shell_wildcard_status
commands_expand(struct run *r, const struct shell_static_entry *cmd,
                const char *pattern)
{
  enum shell_wildcard_status ret_val = SHELL_WILDCARD_CMD_NO_MATCH_FOUND;
  const struct shell_static_entry *entry = NULL;
  struct shell_static_entry dloc;
  size_t cmd_idx = 0;
  size_t cnt = 0;

  while ((entry = cmd_get(NULL, cmd, cmd_idx++, &dloc)) != NULL)
    {
      if (fnmatch(pattern, entry->syntax, 0) == 0)
        {
          ret_val = command_add(r->temp_buff, &r->cmd_tmp_buff_len,
                                entry->syntax, pattern);
          if (ret_val == SHELL_WILDCARD_CMD_MISSING_SPACE)
            {
              fflush(stdout);
              fprintf(stderr, "Command buffer is too short to"
                      " expand all commands matching"
                      " wildcard pattern: %s\n", pattern);
              break;
            }
          else if (ret_val != SHELL_WILDCARD_CMD_ADDED)
            {
              break;
            }

          cnt++;
        }
    }

  if (cnt > 0)
    {
      pattern_remove(r->temp_buff, &r->cmd_tmp_buff_len, pattern);
    }

  return ret_val;
}

/* z_shell_wildcard_process() */

static enum shell_wildcard_status
wildcard_process(struct run *r, const struct shell_static_entry *cmd,
                 const char *pattern)
{
  if (cmd == NULL)
    {
      return SHELL_WILDCARD_NOT_FOUND;
    }

  if (!has_wildcard(pattern))
    {
      return SHELL_WILDCARD_NOT_FOUND;
    }

  return commands_expand(r, cmd, pattern);
}

/* The words of WORDS from FIRST joined by single spaces into BUF (the
 * command buffer after z_shell_spaces_trim()); false if they do not fit.
 */

static bool join_words(char *buf, size_t size, int argc, char **words,
                       int first)
{
  size_t len = 0;
  int i;

  buf[0] = '\0';
  for (i = first; i < argc; i++)
    {
      size_t n = strlen(words[i]);

      if (len + n + (i > first ? 1 : 0) + 1 > size)
        {
          return false;
        }

      if (i > first)
        {
          buf[len++] = ' ';
        }

      memcpy(buf + len, words[i], n + 1);
      len += n;
    }

  return true;
}

/* z_shell_make_argv(&argc, argv, cmd_buff, ARGC_MAX) on the expanded line
 * (words separated by single spaces): the words in place.
 */

static size_t make_argv(char **argv, char *buf)
{
  size_t argc = 0;
  char *p = buf;

  while (*p != '\0' && argc < ARGC_MAX)
    {
      while (*p == ' ')
        {
          p++;
        }

      if (*p == '\0')
        {
          break;
        }

      argv[argc++] = p;
      while (*p != '\0' && *p != ' ')
        {
          p++;
        }

      if (*p == ' ')
        {
          *p++ = '\0';
        }
    }

  argv[argc] = NULL;
  return argc;
}

/* Execution (subsys/shell/shell.c) -----------------------------------------*/

/* active_cmd_prepare() (no raw arguments) */

static void active_cmd_prepare(const struct shell_static_entry *entry,
                               struct shell_static_entry *active_cmd,
                               struct shell_static_entry *help_entry,
                               size_t *lvl, size_t *handler_lvl)
{
  if (entry->handler)
    {
      *handler_lvl = *lvl;
      *active_cmd = *entry;
    }

  if (entry->help)
    {
      *help_entry = *entry;
    }
}

/* exec_cmd() */

static int exec_cmd(struct shell *sh, struct run *r, size_t argc,
                    size_t cmd_lvl,
                    const struct shell_static_entry *help_entry)
{
  size_t cmd_argc = argc - cmd_lvl;
  char **cmd_argv = &r->argv[cmd_lvl];
  int ret_val = 0;

  if (r->active_cmd.handler == NULL)
    {
      if (help_entry->help == NULL)
        {
          return -ENOEXEC;
        }

      if (help_entry->help != r->active_cmd.help)
        {
          r->active_cmd = *help_entry;
        }

      internal_help_print(&r->active_cmd);
      return SHELL_CMD_HELP_PRINTED;
    }

  if (r->active_cmd.args.mandatory)
    {
      uint32_t mand = r->active_cmd.args.mandatory;
      uint8_t opt8 = r->active_cmd.args.optional;
      uint32_t opt = (opt8 == SHELL_OPT_ARG_CHECK_SKIP) ? UINT16_MAX : opt8;

      /* cmd_precheck() */

      if (cmd_argc < mand || cmd_argc > mand + opt)
        {
          fflush(stdout);
          fprintf(stderr, "%s: wrong parameter count\n",
                  r->active_cmd.syntax);
          internal_help_print(&r->active_cmd);
          return -EINVAL;
        }
    }

  fflush(stdout);
  if (sh != NULL)
    {
      sh->cmd_pid = getpid();
    }

  ret_val = r->active_cmd.handler(sh, cmd_argc, cmd_argv);
  fflush(stdout);
  if (sh != NULL)
    {
      sh->cmd_pid = 0;
    }

  return ret_val;
}

/* execute() for the root command ROOT, from nsh's words. */

static int execute(struct shell *sh, struct run *r,
                   const struct shell_static_entry *root, int words_argc,
                   char **words)
{
  struct shell_static_entry dloc = { 0 };  /* Memory for dynamic commands. */
  const struct shell_static_entry *parent = NULL;
  const struct shell_static_entry *entry = NULL;
  struct shell_static_entry help_entry;
  size_t cmd_lvl = 0;
  size_t cmd_with_handler_lvl = 0;
  bool wildcard_found = false;
  bool has_last_handler = false;
  size_t argc = 0;              /* z_shell_make_argv(): 2 while words follow */
  size_t argvp = 0;             /* index of argvp in argv */
  int word = 0;

  memset(&r->active_cmd, 0, sizeof(r->active_cmd));

  /* z_shell_wildcard_prepare(): the line, trimmed */

  if (!join_words(r->temp_buff, sizeof(r->temp_buff), words_argc, words, 0))
    {
      return -ENOEXEC;
    }

  r->cmd_tmp_buff_len = (uint16_t)(strlen(r->temp_buff) + 1);

  help_entry.help = NULL;

  /* Below loop is analyzing subcommands of found root command. */

  while ((argc != 1) && (cmd_lvl < ARGC_MAX))
    {
      if (word >= words_argc)
        {
          return -ENOEXEC;
        }

      /* z_shell_make_argv(&argc, argvp, cmd_buf, 2) */

      argc = (word + 1 < words_argc) ? 2 : 1;
      r->argv[argvp] = words[word];
      r->argv[argvp + 1] = NULL;
      if (argc == 2 &&
          join_words(r->rest, sizeof(r->rest), words_argc, words, word + 1))
        {
          r->argv[argvp + 1] = r->rest;
        }

      word++;

      if (cmd_lvl > 0)
        {
          enum shell_wildcard_status status;

          status = wildcard_process(r, entry, r->argv[argvp]);

          /* Wildcard character found but there is no matching command. */

          if (status == SHELL_WILDCARD_CMD_NO_MATCH_FOUND)
            {
              break;
            }

          /* Wildcard character was not found function can process
           * argument.
           */

          if (status != SHELL_WILDCARD_NOT_FOUND)
            {
              ++cmd_lvl;
              wildcard_found = true;
              continue;
            }
        }

      if (has_last_handler == false)
        {
          entry = find_cmd(root, parent, r->argv[argvp], &dloc);
        }

      argvp++;
      if (entry)
        {
          /* wildcard_check_report() */

          if (wildcard_found && entry->handler)
            {
              fflush(stdout);
              fprintf(stderr,
                      "Error: requested multiple function executions\n");
              return -ENOEXEC;
            }

          active_cmd_prepare(entry, &r->active_cmd, &help_entry, &cmd_lvl,
                             &cmd_with_handler_lvl);
          parent = entry;
        }
      else
        {
          if (cmd_lvl == 0)
            {
              fflush(stdout);
              fprintf(stderr, "%s%s\n", r->argv[0], SHELL_MSG_CMD_NOT_FOUND);
            }

          /* last handler found - no need to search commands in the next
           * iteration.
           */

          has_last_handler = true;
        }

      cmd_lvl++;
    }

  if ((cmd_lvl >= ARGC_MAX) && (argc == 2))
    {
      /* argc == 2 indicates that when command string was parsed there was
       * more characters remaining. It means that number of arguments
       * exceeds the limit.
       */

      fflush(stdout);
      fprintf(stderr, "%s\n", SHELL_MSG_TOO_MANY_ARGS);
      return -ENOEXEC;
    }

  if (wildcard_found)
    {
      /* z_shell_wildcard_finalize(): the expanded line is the command
       * buffer, split again.
       */

      memset(r->argv, 0, sizeof(r->argv));
      cmd_lvl = make_argv(r->argv, r->temp_buff);
    }

  /* Executing the deepest found handler. */

  return exec_cmd(sh, r, cmd_lvl, cmd_with_handler_lvl, &help_entry);
}

int z_shell_cmd_main(const struct shell_static_entry *root, int argc,
                     char **argv)
{
  struct shell *sh = backend_get();
  struct run *r;
  int ret;

  r = calloc(1, sizeof(*r));
  if (r == NULL)
    {
      fprintf(stderr, "%s: out of memory\n", argv[0]);
      return EXIT_FAILURE;
    }

  ret = execute(sh, r, root, argc, argv);
  free(r);
  fflush(stdout);
  return ret == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
