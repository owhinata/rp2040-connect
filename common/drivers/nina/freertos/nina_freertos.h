/* SPDX-License-Identifier: MIT */
/* What the FreeRTOS binding of the shared NINA core (nina_os.c here) needs
 * from the application, besides nina_os.h.
 */

#ifndef NINA_FREERTOS_H_
#define NINA_FREERTOS_H_

/* Makes the binding's mutexes and semaphores (static, no heap).  Called
 * once, before any task can use the NINA core (nina_os.h), from a task or
 * before the scheduler starts; not from an interrupt handler.  A second
 * call, and a use of the core before the first, stop at configASSERT().
 */
void nina_os_initialize(void);

#endif /* NINA_FREERTOS_H_ */
