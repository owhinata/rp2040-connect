/****************************************************************************
 * nuttx/apps/init/nano_telnet.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The Telnet shell: nsh on TCP port CONFIG_NANO_INIT_TELNET_PORT for one
 * client at a time, the counterpart of the Zephyr build's Telnet shell
 * backend (zephyr/subsys/shell/backends/shell_telnet.c,
 * CONFIG_SHELL_BACKEND_TELNET, CONFIG_SHELL_TELNET_PORT=23).
 *
 * As in Zephyr:
 *   - the listening socket is opened once at boot, before main() would run
 *     (telnet_init(): socket, bind to INADDR_ANY, listen with a backlog of
 *     1, no SO_REUSEADDR), whether or not Wi-Fi is connected, and stays
 *     open across Wi-Fi disconnections (the NINA keeps it);
 *   - a client that connects while another one is connected is accepted
 *     and closed at once, "Telnet client already connected." is logged and
 *     the first client is kept (telnet_accept());
 *   - a session ends when the client closes the connection or the
 *     connection fails (telnet_recv() errors, telnet_end_client_connection()),
 *     after which the next client is accepted.
 *
 * NuttX's own Telnet daemon (netutils/telnetd with system/telnetd and
 * NSH_TELNET) is not used: it ends, closing its listening socket, when the
 * driver refuses a session over CONFIG_TELNET_MAXLCLIENTS (the second
 * client with MAXLCLIENTS=1), and when SO_REUSEADDR fails, which it does on
 * the NINA's sockets (usrsock: ENOPROTOOPT); and its sessions print the
 * "NuttShell (NSH)" greeting.
 *
 * The sessions use NuttX's Telnet driver (drivers/net/telnet.c,
 * CONFIG_NETDEV_TELNET): the "/dev/telnet" factory wraps an accepted socket
 * into a character device /dev/telnetN, which an nsh session (the same one
 * as on USB, nano_nsh_session()) uses as fd 0/1/2.  With
 * CONFIG_TELNET_MAXLCLIENTS=1 the driver has a single session slot, held
 * until the last descriptor of the device is closed (the session task and
 * whatever it started in the background have exited).  The factory refuses
 * another session with -ENFILE while the slot is held; that is the "already
 * connected" case.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <semaphore.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/net/telnet.h>

#include "app_log.h"
#include "priorities.h"
#include "nano_init.h"

#ifdef CONFIG_NANO_INIT_TELNET

/* The module name and level of the Zephyr backend
 * (CONFIG_SHELL_TELNET_LOG_LEVEL=3, INF).
 */

LOG_MODULE_REGISTER(shell_telnet, LOG_LEVEL_INF);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The listening socket, and the semaphore the task posts once it has tried
 * to set it up.
 */

static int   g_listensd = -1;
static sem_t g_setup_done = SEM_INITIALIZER(0);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: telnet_setup_server
 *
 * Description:
 *   shell_telnet.c telnet_setup_server() for NET_AF_INET (the Zephyr build
 *   has no IPv6): the same calls, backlog and log messages.  The socket is
 *   SOCK_CLOEXEC so that the sessions do not inherit it.
 *
 ****************************************************************************/

static int telnet_setup_server(void)
{
  struct sockaddr_in addr;
  int ret;
  int fd;

  fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, IPPROTO_TCP);
  if (fd < 0)
    {
      ret = -errno;
      LOG_ERR("Failed to create telnet NET_AF_INET socket");
      goto error;
    }

  memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_port        = HTONS(CONFIG_NANO_INIT_TELNET_PORT);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(fd, (FAR struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      ret = -errno;
      LOG_ERR("Cannot bind on family NET_AF_INET (%d)", ret);
      goto error;
    }

  if (listen(fd, 1) < 0)
    {
      ret = -errno;
      LOG_ERR("Cannot listen on family NET_AF_INET (%d)", ret);
      goto error;
    }

  g_listensd = fd;
  return 0;

error:
  LOG_ERR("Unable to start telnet on NET_AF_INET (%d)", ret);

  if (fd >= 0)
    {
      close(fd);
    }

  return ret;
}

/****************************************************************************
 * Name: telnet_null_stdio
 *
 * Description:
 *   Points fd 0/1/2 of this task at /dev/null.  They stay taken, so that
 *   the listening and the accepted sockets never get one of them (a
 *   session's descriptors are put there with dup2()).
 *
 ****************************************************************************/

static void telnet_null_stdio(void)
{
  int fd = open("/dev/null", O_RDWR);
  int i;

  if (fd < 0)
    {
      return;
    }

  for (i = 0; i < 3; i++)
    {
      if (fd != i)
        {
          dup2(fd, i);
        }
    }

  if (fd > 2)
    {
      close(fd);
    }
}

/****************************************************************************
 * Name: telnet_start_session
 *
 * Description:
 *   Wraps the accepted socket sd into a Telnet character device and starts
 *   an nsh session on it, the way nuttx-apps/netutils/telnetd/
 *   telnetd_daemon.c does.  Returns -ENFILE when a session is already
 *   running.  On success the driver has taken over sd (and closed it); on
 *   failure the caller closes it.
 *
 ****************************************************************************/

static int telnet_start_session(int sd)
{
  struct telnet_session_s session;
  pid_t pid;
  int factory;
  int fd;
  int ret;

  factory = open("/dev/telnet", O_RDONLY);
  if (factory < 0)
    {
      return -errno;
    }

  session.ts_sd         = sd;
  session.ts_devpath[0] = '\0';

  ret = ioctl(factory, SIOCTELNET, (unsigned long)((uintptr_t)&session));
  ret = ret < 0 ? -errno : 0;
  close(factory);
  if (ret < 0)
    {
      return ret;
    }

  /* The driver now holds the connection; sd is closed. */

  fd = open(session.ts_devpath, O_RDWR);
  if (fd < 0)
    {
      LOG_ERR("Cannot open %s (%d)", session.ts_devpath, -errno);
      return 0;
    }

  dup2(fd, 0);
  dup2(fd, 1);
  dup2(fd, 2);
  if (fd > 2)
    {
      close(fd);
    }

  /* The session inherits fd 0/1/2 (the listening socket is CLOEXEC). */

  pid = task_create("nsh_telnet", CONFIG_NANO_INIT_NSH_PRIORITY,
                    CONFIG_NANO_INIT_NSH_STACKSIZE, nano_nsh_session, NULL);
  if (pid < 0)
    {
      LOG_ERR("Cannot start the Telnet session (%d)", -errno);
    }

  /* The session has its own copies; dropping ours ends the connection if
   * the session could not be started.
   */

  telnet_null_stdio();
  return 0;
}

/****************************************************************************
 * Name: telnet_main
 *
 * Description:
 *   The task that accepts clients: shell_telnet.c telnet_accept(), which the
 *   Zephyr build runs in its socket service thread (net_socket_service).
 *
 ****************************************************************************/

static int telnet_main(int argc, FAR char *argv[])
{
  int ret;

  /* No console of its own: /dev/null while no session is being started. */

  telnet_null_stdio();

  ret = telnet_setup_server();
  if (ret == 0)
    {
      LOG_INF("Telnet shell backend initialized");
    }

  sem_post(&g_setup_done);
  if (ret < 0)
    {
      return EXIT_FAILURE;
    }

  for (; ; )
    {
      int sd;

      sd = accept4(g_listensd, NULL, NULL, SOCK_CLOEXEC);
      if (sd < 0)
        {
          if (errno != EINTR)
            {
              LOG_ERR("Telnet accept error (%d)", -errno);

              /* Zephyr returns to poll() on the listening socket; do not
               * turn a lasting error into a busy loop.
               */

              usleep(CONFIG_NANO_INIT_RETRY_MS * 1000);
            }

          continue;
        }

      ret = telnet_start_session(sd);
      if (ret == 0)
        {
          continue;
        }

      if (ret == -ENFILE)
        {
          /* Too many connections. */

          LOG_ERR("Telnet client already connected.");
        }
      else
        {
          LOG_ERR("Cannot start the Telnet session (%d)", ret);
        }

      close(sd);
    }

  return EXIT_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nano_telnet_start
 ****************************************************************************/

void nano_telnet_start(void)
{
  pid_t pid;

  pid = task_create("telnetd", NANO_PRIO_TELNET_SERVICE,
                    NANO_STACK_TELNET_SERVICE, telnet_main, NULL);
  if (pid < 0)
    {
      LOG_ERR("Unable to start telnet on NET_AF_INET (%d)", -errno);
      return;
    }

  while (sem_wait(&g_setup_done) < 0 && errno == EINTR)
    {
    }
}

#endif /* CONFIG_NANO_INIT_TELNET */
