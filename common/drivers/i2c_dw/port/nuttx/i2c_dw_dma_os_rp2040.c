/* SPDX-License-Identifier: MIT */
/* The NuttX part of the DMA reads of I2C0: the boundary of
 * common/drivers/i2c_dw/port/i2c_dw_dma_os.h on NuttX's own DMA driver
 * (arch/arm/src/rp2040/rp2040_dmac.c, CONFIG_RP2040_DMAC), which other code
 * of this build may use as usual.
 *
 *   the boundary                  here
 *   i2c_dw_dma_os_initialize()    rp2040_dmachannel_trywait() twice,
 *                                 once at start-up (the local patch
 *                                 nuttx/patches/nuttx/0011: never waits,
 *                                 NULL at once when no channel is free or
 *                                 another task holds the allocator's
 *                                 lock, where rp2040_dmachannel() waits);
 *                                 the two channels stay this file's.
 *                                 Without both, the one it got is freed
 *                                 again and it returns -EBUSY, and does
 *                                 not try again: I2C0 starts without the
 *                                 DMA and reads on the interrupts
 *   _request_channel()            one of the two that is not taken by the
 *                                 read in progress, or -EBUSY; never waits
 *   _release_channel()            it is free for the next read (the channel
 *                                 stays allocated; nothing is written)
 *   _config_tx()                  rp2040_txdmasetup(): DREQ 32, halfwords,
 *                                 the read address incrementing
 *   _config_rx()                  rp2040_rxdmasetup(): DREQ 33, bytes, the
 *                                 write address incrementing
 *   _start()                      the channel's flag cleared (INTS0), its
 *                                 interrupt enabled (INTE0), then
 *                                 rp2040_dmastart() with this file's
 *                                 callback
 *   _stop()                       its interrupt disabled, then its bit
 *                                 written to CHAN_ABORT, without waiting
 *                                 (rp2040_dmastop() waits on CHAN_ABORT
 *                                 without a bound; the core waits with one)
 *   _is_busy()                    BUSY of its CTRL_TRIG
 *                                 (rp2040_dma_register()), or its bit still
 *                                 set in CHAN_ABORT
 *   _acknowledge()                its bit written to INTS0 (write 1 to
 *                                 clear)
 *
 * The register writes of the channel's own bit (INTE0 through the atomic
 * set and clear aliases, INTS0 and CHAN_ABORT, which are write-1) leave the
 * other channels alone; rp2040_dmac.c itself writes INTE0 and INTS0 the
 * same way (INTS0 with the local patches nuttx/patches/nuttx/0011 and
 * 0012).  rp2040_dmachannel_trywait() enables the channel's interrupt;
 * this file keeps it disabled between reads, as Zephyr's driver does.
 *
 * The interrupt: rp2040_dmac.c owns DMA_IRQ_0 (arm_dma_initialize(), from
 * up_initialize(), before the board's bringup).  Its handler clears every
 * flag of INTS0 and calls, for each flagged channel, the callback given to
 * rp2040_dmastart() once, with the status EIO when the channel's AHB_ERROR
 * is set (after it has written READ_ERROR and WRITE_ERROR back through the
 * set alias of CTRL_TRIG, a trigger register: a channel that stopped on
 * an error may start again from where it stopped).  This file's callback
 * then requests the channel's abort at once (CHAN_ABORT), so that such a
 * restart ends at once, and passes -EIO, or 0, to the core.  NuttX's handler
 * keeps the status of an earlier channel of the same interrupt for the
 * later ones, so a bus error of another user's channel served in the same
 * interrupt ends this read with -EIO too (a failed read, counted as an
 * error, not lost data).
 *
 * DMA_IRQ_0 and I2C0_IRQ have the same priority (CONFIG_ARCH_IRQPRIO off:
 * every interrupt at NVIC_SYSH_PRIORITY_DEFAULT) on one core, so neither
 * handler interrupts the other.  The callback's slot is read with
 * interrupts off in the thread functions (up_irq_save()).
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/rp2040_dma.h"
#include "hardware/rp2040_memorymap.h"
#include "rp2040_dmac.h"

#include "i2c_dw_dma_os.h"

/* The two channels of the reads (TX and RX of one read). */

#define I2C_DW_DMA_CHANNELS 2

struct i2c_dw_dma_slot
{
  DMA_HANDLE handle;
  uint32_t channel;
  bool taken;
  i2c_dw_dma_os_callback_t callback;
  void *user_data;
};

static struct i2c_dw_dma_slot g_slots[I2C_DW_DMA_CHANNELS];
static int g_init_result = 1; /* 1: not yet initialized */

static struct i2c_dw_dma_slot *slot_of(uint32_t channel)
{
  for (int i = 0; i < I2C_DW_DMA_CHANNELS; i++)
    {
      if (g_init_result == 0 && g_slots[i].channel == channel)
        {
          return &g_slots[i];
        }
    }

  return NULL;
}

/* rp2040_dmac.c's callback (DMA_IRQ_0, its handler): status 0, or EIO. */

static void i2c_dw_dma_callback(DMA_HANDLE handle, uint8_t status,
                                void *arg)
{
  struct i2c_dw_dma_slot *slot = arg;
  i2c_dw_dma_os_callback_t callback = slot->callback;

  (void)handle;

  if (status != 0)
    {
      putreg32(1u << slot->channel, RP2040_DMA_CHAN_ABORT);
    }

  if (callback != NULL)
    {
      callback(slot->user_data, slot->channel, status != 0 ? -EIO : 0);
    }
}

int i2c_dw_dma_os_initialize(void)
{
  if (g_init_result <= 0)
    {
      return g_init_result;
    }

  for (int i = 0; i < I2C_DW_DMA_CHANNELS; i++)
    {
      DMA_HANDLE handle = rp2040_dmachannel_trywait();

      if (handle == NULL)
        {
          for (int j = 0; j < i; j++)
            {
              rp2040_dmafree(g_slots[j].handle);
            }

          g_init_result = -EBUSY;
          return g_init_result;
        }

      g_slots[i].handle = handle;
      g_slots[i].channel = (rp2040_dma_register(handle, 0) -
                            RP2040_DMA_CH(0)) /
                           (RP2040_DMA_CH(1) - RP2040_DMA_CH(0));
      clrbits_reg32(1u << g_slots[i].channel, RP2040_DMA_INTE0);
    }

  g_init_result = 0;
  return 0;
}

bool i2c_dw_dma_os_is_ready(void)
{
  return g_init_result == 0;
}

int i2c_dw_dma_os_request_channel(void)
{
  if (g_init_result != 0)
    {
      return -ENODEV;
    }

  for (int i = 0; i < I2C_DW_DMA_CHANNELS; i++)
    {
      if (!g_slots[i].taken)
        {
          irqstate_t flags = up_irq_save();

          g_slots[i].taken = true;
          g_slots[i].callback = NULL;
          g_slots[i].user_data = NULL;
          up_irq_restore(flags);
          return (int)g_slots[i].channel;
        }
    }

  return -EBUSY;
}

void i2c_dw_dma_os_release_channel(uint32_t channel)
{
  struct i2c_dw_dma_slot *slot = slot_of(channel);
  irqstate_t flags;

  if (slot == NULL || !slot->taken)
    {
      return;
    }

  flags = up_irq_save();
  clrbits_reg32(1u << channel, RP2040_DMA_INTE0);
  slot->callback = NULL;
  slot->user_data = NULL;
  slot->taken = false;
  up_irq_restore(flags);
}

static int setup_slot(uint32_t channel, i2c_dw_dma_os_callback_t callback,
                      void *user_data, struct i2c_dw_dma_slot **out)
{
  struct i2c_dw_dma_slot *slot = slot_of(channel);
  irqstate_t flags;

  if (slot == NULL || !slot->taken)
    {
      return -EINVAL;
    }

  if (i2c_dw_dma_os_is_busy(channel))
    {
      return -EBUSY;
    }

  flags = up_irq_save();
  slot->callback = callback;
  slot->user_data = user_data;
  up_irq_restore(flags);
  *out = slot;
  return 0;
}

int i2c_dw_dma_os_config_tx(uint32_t channel, const uint16_t *words,
                            uintptr_t dest, uint32_t count,
                            i2c_dw_dma_os_callback_t callback,
                            void *user_data)
{
  struct i2c_dw_dma_slot *slot;
  dma_config_t config =
  {
    .dreq = RP2040_DMA_DREQ_I2C0_TX,
    .size = RP2040_DMA_SIZE_HALFWORD,
    .noincr = false,
  };

  int ret = setup_slot(channel, callback, user_data, &slot);

  if (ret == 0)
    {
      rp2040_txdmasetup(slot->handle, dest, (uintptr_t)words,
                        count * sizeof(uint16_t), config);
    }

  return ret;
}

int i2c_dw_dma_os_config_rx(uint32_t channel, uintptr_t source,
                            uint8_t *bytes, uint32_t count,
                            i2c_dw_dma_os_callback_t callback,
                            void *user_data)
{
  struct i2c_dw_dma_slot *slot;
  dma_config_t config =
  {
    .dreq = RP2040_DMA_DREQ_I2C0_RX,
    .size = RP2040_DMA_SIZE_BYTE,
    .noincr = false,
  };

  int ret = setup_slot(channel, callback, user_data, &slot);

  if (ret == 0)
    {
      rp2040_rxdmasetup(slot->handle, source, (uintptr_t)bytes, count,
                        config);
    }

  return ret;
}

int i2c_dw_dma_os_start(uint32_t channel)
{
  struct i2c_dw_dma_slot *slot = slot_of(channel);

  if (slot == NULL || !slot->taken)
    {
      return -EINVAL;
    }

  if (i2c_dw_dma_os_is_busy(channel))
    {
      return -EBUSY;
    }

  putreg32(1u << channel, RP2040_DMA_INTS0);
  setbits_reg32(1u << channel, RP2040_DMA_INTE0);
  rp2040_dmastart(slot->handle, i2c_dw_dma_callback, slot);
  return 0;
}

int i2c_dw_dma_os_stop(uint32_t channel)
{
  struct i2c_dw_dma_slot *slot = slot_of(channel);

  if (slot == NULL || !slot->taken)
    {
      return -EINVAL;
    }

  clrbits_reg32(1u << channel, RP2040_DMA_INTE0);
  putreg32(1u << channel, RP2040_DMA_CHAN_ABORT);
  return 0;
}

bool i2c_dw_dma_os_is_busy(uint32_t channel)
{
  struct i2c_dw_dma_slot *slot = slot_of(channel);

  if (slot == NULL)
    {
      return false;
    }

  return (getreg32(rp2040_dma_register(slot->handle,
                                       RP2040_DMA_CTRL_TRIG_OFFSET)) &
          RP2040_DMA_CTRL_TRIG_BUSY) != 0 ||
         (getreg32(RP2040_DMA_CHAN_ABORT) & (1u << channel)) != 0;
}

void i2c_dw_dma_os_acknowledge(uint32_t channel)
{
  if (slot_of(channel) != NULL)
    {
      putreg32(1u << channel, RP2040_DMA_INTS0);
    }
}
