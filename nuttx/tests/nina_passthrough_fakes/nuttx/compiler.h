/* SPDX-License-Identifier: MIT */
/* The little of NuttX's <nuttx/compiler.h> that the board's
 * include/nina_passthrough.h uses (host test nina_passthrough).
 */
#ifndef TEST_FAKE_NUTTX_COMPILER_H
#define TEST_FAKE_NUTTX_COMPILER_H
#define noreturn_function __attribute__((noreturn))
#define FAR
#endif
