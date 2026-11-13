/* SPDX-License-Identifier: Apache-2.0 */
/* The fixed-point types of Zephyr's <zephyr/dsp/types.h> that the sensor
 * API uses.  Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_DSP_TYPES_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_DSP_TYPES_H

#include <stdint.h>

typedef int8_t q7_t;
typedef int16_t q15_t;
typedef int32_t q31_t;
typedef int64_t q63_t;

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_DSP_TYPES_H */
