/****************************************************************************
 * nuttx/apps/init/nano_init.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The init task of the Nano RP2040 Connect port: nsh on the USB CDC ACM
 * console.  The Telnet shell is in nano_telnet.c; init starts it at the
 * point where the Zephyr build's Telnet backend starts (before main()).
 *
 * The Zephyr build runs its shell on the CDC ACM UART from boot, prompts
 * without waiting for anything, keeps running while the host closes and
 * reopens the port, and does not look at DTR (the session here does not
 * either; only cdcacm does, see below).  NuttX's USB console front
 * end (nshlib/nsh_usbconsole.c) waits for three Enter keys before the first
 * prompt, so it is not used.  Instead this task:
 *
 *   1. calls nsh_initialize() once;
 *   2. opens CONFIG_NANO_INIT_CONSOLE, retrying while it fails (cdcacm
 *      refuses the first open with -ENOTCONN until the host has configured
 *      the device) or while the device reports a disconnection (POLLHUP);
 *   3. sets the terminal modes (see console_setup());
 *   4. makes it fd 0/1/2 and starts an nsh session in its own task, the
 *      way nuttx-apps/netutils/telnetd/telnetd_daemon.c starts a Telnet
 *      session;
 *   5. waits for the session to end and closes 0/1/2, then goes back to 2.
 *
 * A session ends when readline() gets an error: the serial upper half
 * returns -ENOTCONN from read() once the CDC ACM driver reports that the
 * USB connection is gone (cable unplugged, bus reset, host unconfigured the
 * device; CONFIG_SERIAL_REMOVABLE).  The host closing and reopening the
 * port does not end it, just as with Zephyr.  While no program on the
 * host has the port open (DTR low), cdcacm drops the output instead of
 * keeping it for the next program (nuttx/patches/nuttx/0002), so the
 * prompt of a session started before the host opened the port is never
 * sent; the first prompt comes with the first input (e.g. Enter or the
 * Ctrl-C that the host tools send).
 *
 * The session is nshlib's nsh_session() with NSH_LOGIN_NONE, which is what
 * nsh_consolemain() does except that it passes NSH_LOGIN_LOCAL and so
 * prints the "NuttShell (NSH)" greeting first.  The Zephyr shell shows only
 * its prompt.  nsh_newconsole() and nsh_session() are nshlib internals
 * (nsh.h, nsh_console.h); see nuttx-apps/nshlib/nsh_consolemain.c when
 * moving to another NuttX release.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sched.h>
#include <stdbool.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <nshlib/nshlib.h>

#include "nsh.h"
#include "nsh_console.h"

#include "app_log.h"
#include "priorities.h"
#include "nano_init.h"

#ifdef CONFIG_NANO_KV_STORE
#  include "kv_store.h"
#endif

#if defined(CONFIG_NANO_APP_IMU) || defined(CONFIG_NANO_NINA_NET) || \
    defined(CONFIG_NANO_WIFI_PROFILE) || defined(CONFIG_NANO_APP_SENSOR)
#  include "zephyr_compat.h"
#endif

#ifdef CONFIG_NANO_APP_IMU
#  include "imu_cal.h"
#  include "time_sync.h"
#endif

/* nuttx/apps/wifi: <zephyr/net/wifi_credentials.h> and <wifi_profile.h>
 * declare these, but this file does not include the Zephyr compatibility
 * layer's headers (see zephyr_compat.h).
 */

#ifdef CONFIG_NANO_WIFI_CREDENTIALS
int wifi_credentials_init(void);
#endif
#ifdef CONFIG_NANO_WIFI_PROFILE
int wifi_profile_init(void);
#endif

/* The module name and level of the Zephyr build's main.c. */

LOG_MODULE_REGISTER(usb_console, LOG_LEVEL_INF);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nano_nsh_session
 *
 * Description:
 *   The nsh session task.  Same as nsh_consolemain() but without the login
 *   greeting.  Runs on the fd 0/1/2 it inherits from init (USB) or from the
 *   Telnet task (nano_telnet.c).
 *
 ****************************************************************************/

int nano_nsh_session(int argc, FAR char *argv[])
{
  FAR struct console_stdio_s *pstate = nsh_newconsole(true);
  int ret;

  if (pstate == NULL)
    {
      return EXIT_FAILURE;
    }

  ret = nsh_session(pstate, NSH_LOGIN_NONE, argc, argv);

  /* Frees pstate and exits the task. */

  nsh_exit(&pstate->cn_vtbl, ret);
  return ret;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: console_connected
 *
 * Description:
 *   False while the CDC ACM device reports a lost connection.  The serial
 *   upper half sets POLLERR|POLLHUP for a disconnected removable device; a
 *   session started then would end at its first read.
 *
 ****************************************************************************/

static bool console_connected(int fd)
{
  struct pollfd pfd;

  pfd.fd      = fd;
  pfd.events  = POLLIN;
  pfd.revents = 0;

  if (poll(&pfd, 1, 0) < 0)
    {
      return false;
    }

  return (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) == 0;
}

/****************************************************************************
 * Name: console_setup
 *
 * Description:
 *   Terminal modes of the console.  The CDC ACM device is not a NuttX
 *   console (CDCACM_CONSOLE is off), so uart_register() leaves all modes
 *   cleared (drivers/serial/serial.c); set them here.
 *
 *   Output: OPOST|ONLCR, "\n" goes out as "\r\n" (as on a NuttX console
 *   and on the Zephyr shell).
 *
 *   Echo: ECHO.  While readline reads a line it turns ECHO off and echoes
 *   itself the characters it accepts and the line end
 *   (nuttx/patches/apps/0001); while a command runs, the serial driver
 *   echoes.  ICANON is what a console has; readline turns it off while it
 *   reads a line.
 *
 *   Input line ends: no translation (no ICRNL, INLCR, IGNCR).  readline
 *   gets CR and LF as they are and ends a line at either; CR LF and LF CR
 *   are one Enter (nuttx/patches/apps/0002), as in the Zephyr shell.  The
 *   host tools send "\r\n" (host/imu_host/board.py), terminal programs "\r".
 *
 *   No ISIG: CONFIG_TTY_SIGINT is off and no signal is ever sent.  Ctrl-C
 *   (0x03) at the prompt drops the line being typed (patches/apps/0003);
 *   while a command reads the console in canonical mode, Ctrl-C and
 *   Ctrl-D end that read (patches/nuttx/0003); while a builtin application
 *   runs, it waits in the buffer for the next prompt.  Nothing is killed.
 *
 ****************************************************************************/

static void console_setup(int fd)
{
  struct termios tio;

  if (tcgetattr(fd, &tio) < 0)
    {
      return;
    }

  tio.c_iflag &= ~(ICRNL | INLCR | IGNCR);
  tio.c_oflag  = OPOST | ONLCR;
  tio.c_lflag &= ~(ISIG);
  tio.c_lflag |= ECHO | ICANON;

  tcsetattr(fd, TCSANOW, &tio);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nano_init_main
 *
 * Description:
 *   CONFIG_INIT_ENTRYPOINT.  Never returns.
 *
 *   Starts at CONFIG_INIT_PRIORITY, the priority of the Zephyr build's
 *   main() (NANO_PRIO_MAIN, 118), does what that main() does, then drops to
 *   the shell's priority (NANO_PRIO_SHELL, 104) for the console loop, which
 *   the Zephyr build runs in its shell thread.
 *
 ****************************************************************************/

int nano_init_main(int argc, FAR char *argv[])
{
  struct sched_param param;
#if defined(CONFIG_NANO_KV_STORE) || defined(CONFIG_NANO_APP_IMU)
  int ret;
#endif

  nsh_initialize();

#ifdef CONFIG_NANO_WIFI_CREDENTIALS
  /* The Wi-Fi credentials library's SYS_INIT(POST_KERNEL), which Zephyr
   * runs before the threads below start: reads the stored SSIDs.
   */

  wifi_credentials_init();
#endif

  /* Zephyr starts the threads of K_THREAD_DEFINE() before main()
   * (z_init_static_threads()).  Each waits for its work at once.
   */

#ifdef CONFIG_NANO_NINA_NET
  K_THREAD_DEFINE_START(nina_scan_tid);     /* drivers/nina/nina_net.c */
  K_THREAD_DEFINE_START(nina_connect_tid);
  K_THREAD_DEFINE_START(nina_monitor_tid);
#endif
#ifdef CONFIG_NANO_APP_IMU
  K_THREAD_DEFINE_START(imu_stream_tid);  /* imu_stream.c */
#endif
#ifdef CONFIG_NANO_IMU_RAW_MQTT
  K_THREAD_DEFINE_START(imu_raw_mqtt_tid);  /* ../imu/imu_raw_mqtt.c */
#endif
#ifdef CONFIG_NANO_TIME_SYNC
  K_THREAD_DEFINE_START(time_sync_tid);   /* ../imu/time_sync.c */
#endif
#ifdef CONFIG_NANO_WIFI_PROFILE
  K_THREAD_DEFINE_START(wifi_reconnect_tid);  /* nuttx/apps/wifi/wifi_profile.c */
#endif
#ifdef CONFIG_NANO_APP_SENSOR
  K_THREAD_DEFINE_START(sensor_shell_processing_tid);  /* ../sensor/sensor_shell_stream.c */
#endif

#ifdef CONFIG_NANO_INIT_TELNET
  /* The Telnet shell backend's SYS_INIT(APPLICATION), before main(): the
   * listening socket (shell_telnet.c telnet_init()).
   */

  nano_telnet_start();
#endif

  /* zephyr/app/usb-console/src/main.c */

  LOG_INF("USB console started");
#ifdef CONFIG_NANO_KV_STORE
  /* The board has mounted /dev/config already (nano_flash_kv.c); this
   * only checks that it is there, at the same point as main().
   */

  ret = kv_store_init();
  if (ret)
    {
      LOG_ERR("Could not initialize settings: %d", ret);
    }
#endif

#ifdef CONFIG_NANO_APP_IMU
  ret = imu_cal_init();
  if (ret)
    {
      LOG_ERR("Could not set up the IMU calibration: %d", ret);
    }

  time_sync_init();
#endif

#ifdef CONFIG_NANO_WIFI_PROFILE
  (void)wifi_profile_init();
#endif

  param.sched_priority = NANO_PRIO_SHELL;
  sched_setparam(0, &param);

  for (; ; )
    {
      int status;
      pid_t pid;
      int fd;

      /* No console is open, so this is normally fd 0. */

      fd = open(CONFIG_NANO_INIT_CONSOLE, O_RDWR);
      if (fd < 0)
        {
          usleep(CONFIG_NANO_INIT_RETRY_MS * 1000);
          continue;
        }

      if (!console_connected(fd))
        {
          close(fd);
          usleep(CONFIG_NANO_INIT_RETRY_MS * 1000);
          continue;
        }

      console_setup(fd);

      dup2(fd, 0);
      dup2(fd, 1);
      dup2(fd, 2);
      if (fd > 2)
        {
          close(fd);
        }

      /* The new task inherits fd 0/1/2. */

      pid = task_create("nsh", CONFIG_NANO_INIT_NSH_PRIORITY,
                        CONFIG_NANO_INIT_NSH_STACKSIZE,
                        nano_nsh_session, NULL);
      if (pid >= 0)
        {
          while (waitpid(pid, &status, 0) < 0 && errno == EINTR)
            {
            }
        }

      close(0);
      close(1);
      close(2);

      usleep(CONFIG_NANO_INIT_RETRY_MS * 1000);
    }

  return EXIT_FAILURE;
}
