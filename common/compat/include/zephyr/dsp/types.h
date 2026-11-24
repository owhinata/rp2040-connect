/* SPDX-License-Identifier: Apache-2.0 */
/* The fixed-point types of Zephyr's <zephyr/dsp/types.h> that the sensor
 * API uses.  Part of the small Zephyr compatibility layer of the port
 * (common/compat).
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_DSP_TYPES_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_DSP_TYPES_H

#include <stdint.h>

typedef int8_t q7_t;
typedef int16_t q15_t;
typedef int32_t q31_t;
typedef int64_t q63_t;

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_DSP_TYPES_H */
