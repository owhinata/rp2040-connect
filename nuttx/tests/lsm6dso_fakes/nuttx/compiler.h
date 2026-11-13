/* SPDX-License-Identifier: Apache-2.0 */
/* The little of NuttX's <nuttx/compiler.h> that the board's board.h uses
 * (the host test lsm6dso_driver includes the real board.h).
 */
#ifndef TEST_FAKE_NUTTX_COMPILER_H
#define TEST_FAKE_NUTTX_COMPILER_H
#define noreturn_function __attribute__((noreturn))
#define FAR
#endif
