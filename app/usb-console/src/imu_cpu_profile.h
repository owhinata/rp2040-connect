/* SPDX-License-Identifier: Apache-2.0 */
#ifndef IMU_CPU_PROFILE_H_
#define IMU_CPU_PROFILE_H_
#include <stdint.h>
#include <zephyr/shell/shell.h>
#ifdef CONFIG_IMU_CPU_PROFILE
void imu_cpu_profile_start(void);
void imu_cpu_profile_stop(void);
void imu_cpu_profile_print(const struct shell *shell, uint64_t total_cycles, uint64_t busy_cycles);
#else
static inline void imu_cpu_profile_start(void)
{
}
static inline void imu_cpu_profile_stop(void)
{
}
static inline void imu_cpu_profile_print(const struct shell *shell, uint64_t total_cycles,
					 uint64_t busy_cycles)
{
	(void)shell;
	(void)total_cycles;
	(void)busy_cycles;
}
#endif
#endif
