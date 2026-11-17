/* SPDX-License-Identifier: Apache-2.0 */
/* The NuttX part of the Zephyr shell's command handling
 * (common/shell/zshell.c, which includes this file first through
 * common/shell/zshell_os.h): the shell backends.
 *
 * Backends: a struct shell per terminal (<zephyr/shell/shell.h>, NuttX
 * part nuttx/drivers/compat/zephyr_os_shell.h: the path of the command's
 * stdout, /dev/ttyACM0 for USB, /dev/telnet0 for Telnet, found by
 * F_GETPATH or by the inode number among /dev), kept for the life of the
 * system as Zephyr's shell instances are.  Output to it from the command
 * itself goes to its stdout and stderr; output from any other task or
 * thread (after the command has returned) opens the terminal, writes the
 * text with one write() and closes it again, so that no descriptor of a
 * Telnet session is held between two lines: when the client goes, the
 * session's device goes with its last descriptor (the next client gets the
 * slot), the output is dropped while no client is connected (open() fails)
 * and reaches the next client, as Zephyr's Telnet backend does.
 *
 * Every command is a task of its own (an nsh builtin application), so the
 * command that runs on a backend is known by its pid.  On the host (the
 * tests) the path of stdout comes from /proc/self/fd/1.
 */

#ifndef __NUTTX_APPS_SUBCMD_ZSHELL_OS_IMPL_H
#define __NUTTX_APPS_SUBCMD_ZSHELL_OS_IMPL_H

#ifdef __NuttX__
#  include <nuttx/config.h>
#endif

#include <sys/stat.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <zephyr/shell/shell.h>

/* Backends: USB, Telnet and a few redirected outputs. */

#define BACKENDS        4

/* A line that fits is written with one write(). */

#define LINE_MAX_ONCE   256

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

static struct shell *zshell_os_backend_get(void)
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

/* The command's task runs its handler on SH (and SH's output from that
 * task goes to its stdout and stderr), until zshell_os_cmd_exit().
 */

static inline void zshell_os_cmd_enter(struct shell *sh)
{
  sh->cmd_pid = getpid();
}

static inline void zshell_os_cmd_exit(struct shell *sh)
{
  sh->cmd_pid = 0;
}

#endif /* __NUTTX_APPS_SUBCMD_ZSHELL_OS_IMPL_H */
