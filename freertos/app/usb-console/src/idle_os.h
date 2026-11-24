/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the idle pass (common/imu/port/pico_sdk/
 * idle.c, which describes it): FreeRTOS's idle task calls
 * vApplicationIdleHook() in its loop (configUSE_IDLE_HOOK 1, task.h declares
 * it); without tickless idle (configUSE_TICKLESS_IDLE 0) the RP2040 port
 * never waits with WFI itself, so the pass is the hook.
 */

#ifndef IDLE_OS_H
#define IDLE_OS_H

#include "FreeRTOS.h"
#include "task.h"

#define IDLE_OS_HOOK vApplicationIdleHook

#endif /* IDLE_OS_H */
