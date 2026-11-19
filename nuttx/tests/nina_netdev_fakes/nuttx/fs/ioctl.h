/* SPDX-License-Identifier: Apache-2.0 */
/* The wireless ioctl numbers of NuttX's <nuttx/fs/ioctl.h>, for NuttX's
 * <nuttx/wireless/wireless.h> in tests/nina_netdev.c.
 */

#ifndef TEST_NINA_NETDEV_NUTTX_FS_IOCTL_H
#define TEST_NINA_NETDEV_NUTTX_FS_IOCTL_H

#define _WLIOCBASE   (0x8b00)
#define _IOC(type,nr) ((type)|(nr))
#define _WLIOC(nr)   _IOC(_WLIOCBASE,nr)

#endif /* TEST_NINA_NETDEV_NUTTX_FS_IOCTL_H */
