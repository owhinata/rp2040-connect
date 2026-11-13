/* SPDX-License-Identifier: Apache-2.0 */
/* "irqstack": the use of the interrupt stack since boot, as one line in the
 * format of the "IRQ 00" line of the Zephyr build's "kernel thread stacks"
 * (shell_stack_dump()/shell_isr_stack_dump() of Zephyr's
 * subsys/shell/modules/kernel_service/thread/stacks.c), for example
 *
 *   0x20001bc8 IRQ 00                           (real size 2048):\tunused 1684\tusage  364 / 2048 (17 %)
 *
 * NuttX's ps does not show the interrupt stack.  The address is the lowest
 * address of the stack (up_get_intstackbase()); the use is the stack
 * coloration's high-water mark (up_check_intstack(), the same value as the
 * "irq" row of the crash report's task table).  Arguments are ignored.
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

int main(int argc, FAR char *argv[])
{
  size_t size = CONFIG_ARCH_INTERRUPTSTACK;
  size_t used = up_check_intstack(0, 0);
  size_t unused = used < size ? size - used : 0;

  (void)argc;
  (void)argv;

  printf("%p %-32s (real size %4zu):\tunused %4zu\tusage %4zu / %4zu "
         "(%2zu %%)\n", (FAR void *)up_get_intstackbase(0), "IRQ 00", size,
         unused, used, size, used * 100 / size);
  return 0;
}
