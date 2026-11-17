/* SPDX-License-Identifier: MIT */
/* The OS part of the RTIO work queue (common/compat/zephyr_rtio_workq.c)
 * for host tests: no thread.  The test, or the fake kernel
 * (zephyr_kernel_host.c), runs the queued requests with
 * rtio_workq_process().
 */
