/* SPDX-License-Identifier: MIT */
/* The Pico SDK's locks and sleeps on ThreadX: the SDK's RTOS adapter header
 * (PICO_CONFIG_RTOS_ADAPTER_HEADER, which pico/config.h includes in every
 * source of the build, assembly too; pico_sdk_interop.cmake sets it).  It
 * overrides the hooks of pico/lock_core.h, as FreeRTOS's RP2040 port does
 * with its freertos_sdk_config.h (configSUPPORT_PICO_SYNC_INTEROP):
 *
 *   lock_owner_id_t              32 bits: the owner of an SDK mutex is the
 *   lock_get_caller_owner_id()   calling ThreadX thread (its TX_THREAD), or,
 *                                in an interrupt handler, twice the exception
 *                                number; 0 before the scheduler runs (the
 *                                SDK's own id of core 0).  Threads are told
 *                                apart: with the SDK's ids (the core
 *                                number) a second thread on the same core
 *                                would take a held mutex for its own
 *                                ("deadlock") and mutex_try_enter_block_until()
 *                                would give up at once (stdio_usb drops the
 *                                text or the input then)
 *   lock_internal_spin_unlock_with_wait(),
 *   lock_internal_spin_unlock_with_best_effort_wait_or_timeout()
 *                                a thread blocks in ThreadX until the lock
 *                                is notified (or the timeout), instead of
 *                                WFE, so lower priorities run meanwhile
 *   lock_internal_spin_unlock_with_notify()
 *                                wakes them, from a thread or an interrupt
 *                                handler
 *
 * Where a thread cannot block in ThreadX (before the scheduler runs, in an
 * interrupt handler, with interrupts masked by the caller, inside ThreadX's
 * preemption lock), the SDK's own behaviour is kept (pico_sdk_interop.c).
 * With the waits overridden, sleep_until() and so sleep_ms() and sleep_us()
 * wait on the SDK's sleep notifier (PICO_TIME_USE_SLEEP_NOTIFIER, pico_time),
 * which the alarm notifies just before the time, and spin the rest of the
 * way.  sync_internal_yield_until_before() is not overridden: the SDK calls
 * it only without the default alarm pool, which this build has.
 *
 * Also as with FreeRTOS: malloc() is serialized with an SDK mutex
 * (PICO_USE_MALLOC_MUTEX; without pico_multicore the SDK leaves it out),
 * and the sleep's spin at its end is 150 us (PICO_TIME_SLEEP_OVERHEAD_ADJUST_US;
 * 6 by default), which covers the switch to the woken thread.
 */

#ifndef THREADX_DRIVERS_PICO_SDK_INTEROP_PICO_SDK_INTEROP_H
#define THREADX_DRIVERS_PICO_SDK_INTEROP_PICO_SDK_INTEROP_H

#ifndef PICO_USE_MALLOC_MUTEX
#define PICO_USE_MALLOC_MUTEX 1
#endif

#ifndef PICO_TIME_SLEEP_OVERHEAD_ADJUST_US
#define PICO_TIME_SLEEP_OVERHEAD_ADJUST_US 150
#endif

#ifndef __ASSEMBLER__

#include <stdbool.h>
#include <stdint.h>

struct lock_core;

uint32_t pico_sdk_interop_owner_id(void);
void pico_sdk_interop_wait(struct lock_core *lock, uint32_t save);
void pico_sdk_interop_notify(struct lock_core *lock, uint32_t save);
bool pico_sdk_interop_wait_until(struct lock_core *lock, uint32_t save, uint64_t until_us);

#define lock_owner_id_t            uint32_t
#define LOCK_INVALID_OWNER_ID      UINT32_MAX
#define lock_get_caller_owner_id() pico_sdk_interop_owner_id()

#define lock_internal_spin_unlock_with_wait(lock, save)   pico_sdk_interop_wait(lock, save)
#define lock_internal_spin_unlock_with_notify(lock, save) pico_sdk_interop_notify(lock, save)
#define lock_internal_spin_unlock_with_best_effort_wait_or_timeout(lock, save, until)              \
	pico_sdk_interop_wait_until(lock, save, to_us_since_boot(until))

#endif /* __ASSEMBLER__ */

#endif /* THREADX_DRIVERS_PICO_SDK_INTEROP_PICO_SDK_INTEROP_H */
