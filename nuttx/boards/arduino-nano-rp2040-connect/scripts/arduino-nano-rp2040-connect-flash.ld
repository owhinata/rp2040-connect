/****************************************************************************
 * boards/arduino-nano-rp2040-connect/scripts/arduino-nano-rp2040-connect-flash.ld
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* Derived from boards/arm/rp2040/raspberrypi-pico/scripts/
 * raspberrypi-pico-flash.ld (NuttX 13.1.0).  Changes:
 *
 * - flash is the 16 MiB AT25SF128A minus its last 8 KiB
 *   (0x10ffe000-0x10ffffff), which is kept for the key-value store at the
 *   same place as the Zephyr build's storage_partition
 *   (zephyr/app/usb-console/app.overlay).  An image that would reach it
 *   fails to link ("region `flash' overflowed").
 * - .noinit (NOLOAD) between .data and .bss: the startup code neither
 *   zeroes nor copies it (rp2040_start.c works on _sbss.._ebss and
 *   _sdata.._edata only).  _snoinit/_enoinit mark its bounds.  "AT > sram"
 *   keeps its load address equal to its RAM address instead of following
 *   .data into flash.
 * - Code in RAM: the RP2040 executes the flash through a 16 KiB cache
 *   (XIP), and the code on the 6667 Hz path of the sensor and on the send
 *   path does not fit in it together.  The objects listed in .text's
 *   EXCLUDE_FILE (between "RAM code begin" and "RAM code end", the only
 *   list) leave their .text and .text.* to .data, whose "*(.text
 *   .text.*)" takes what is left; rp2040_start.c copies .data to RAM
 *   with the rest of it.  Their read-only data stays in flash.  What must
 *   not be listed: code that runs before that copy (rp2040_start.c,
 *   rp2040_clockconfig(), rp2040_boardearlyinitialize() and what they
 *   call: rp2040_gpio.o, divider_hardware.o); code that executes "svc 0"
 *   (on ARMv6-M an svc with interrupts masked becomes a hard fault, which
 *   arm_hardfault.c forwards only from _stext.._etext); the fault path
 *   (the exception entry, arm_hardfault(), _assert() and what they call,
 *   libgcc's helpers among them), so that a fault report does not depend
 *   on code in RAM.  nuttx/script/build.sh checks the list, these rules
 *   and the heap that is left after the link
 *   (nuttx/script/check_ram_code.py).
 *
 * This script also links boot2 (arch/arm/src/rp2040/boot2/Make.defs), whose
 * code is in .text; keep it free of ASSERTs that only hold for NuttX.
 */

MEMORY
{
  flash (rx) : ORIGIN = 0x10000000, LENGTH = 16376K
  sram (rwx) : ORIGIN = 0x20000000, LENGTH = 264K
}

/* The reserved key-value area (not part of any output section). */

__kv_start = 0x10ffe000;
__kv_end   = 0x11000000;

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)

SECTIONS
{
    .flash_begin : {
        __flash_binary_start = .;
    } > flash

    .boot2 : {
        __boot2_start__ = .;
        KEEP (*(.boot2))
        __boot2_end__ = .;
    } > flash

    /* The code that runs from RAM (see "Code in RAM" at the top): the
     * code (.text) of these objects is left out here and taken by .data
     * below.  Their read-only data stays here (.rodata).
     * nuttx/script/build.sh checks the placement after the link.
     */

    .text : {
        _stext = ABSOLUTE(.);
        *(.vectors)
        EXCLUDE_FILE(
            /* RAM code begin */

            /* The Zephyr compatibility layer under the drivers and the
             * application: k_sem, k_poll, the timeouts, k_mutex, k_work,
             * RTIO and its work queue, the memory blocks of the FIFO's
             * buffers, the sensor clock, gpio0 (the LSM6DSOX's INT1 and
             * the NINA's READY) with the board's GPIO glue, the I2C API
             * that the LSM6DSOX's reads go through.
             */

            *libexternal.a:zephyr_kernel.o
            *libexternal.a:zephyr_mutex.o
            *libexternal.a:zephyr_work.o
            *libexternal.a:zephyr_rtio.o
            *libexternal.a:zephyr_rtio_workq.o
            *libexternal.a:zephyr_mem_blocks.o
            *libexternal.a:zephyr_sensor_clock.o
            *libexternal.a:zephyr_gpio.o
            *libexternal.a:zephyr_i2c.o
            *libboard.a:rp2040_gpio_os.o

            /* I2C0 with its DMA reads (the FIFO's status and words) and
             * NuttX's DMA driver, also used by the NINA's SPI.
             */

            *libexternal.a:i2c_dw.o
            *libexternal.a:i2c_dw_dma_os_rp2040.o
            *libarch.a:rp2040_dmac.o

            /* The LSM6DSOX: the FIFO stream (status, read, word parse,
             * gyro bias), its RTIO and work item.  Not the ST register
             * driver (st_lsm6dso_reg.o), which only configures.
             */

            *libexternal.a:lsm6dso.o
            *libexternal.a:lsm6dso_stream.o
            *libexternal.a:lsm6dso_fifo.o
            *libexternal.a:lsm6dso_cal.o
            *libexternal.a:lsm6dso_rtio_stream.o
            *libexternal.a:lsm6dso_rtio.o
            *libexternal.a:lsm6dso_os.o
            *libexternal.a:lsm6dso_sensor.o
            *libexternal.a:stmemsc_i2c.o

            /* The IMU service's side of the sensor: the stream thread,
             * the batches and their queue, the time stamps.  Not
             * time_sync.c (mostly the NTP exchange and its thread) nor
             * imu_cal.c (read at the start).
             */

            *libapps.a:imu_stream.c.*.o
            *libapps.a:imu_raw_batch.c.*.o
            *libapps.a:imu_raw_queue.c.*.o
            *libapps.a:imu_raw_time.c.*.o
            *libapps.a:imu_tick_clock.c.*.o
            *libapps.a:time_offset.c.*.o

            /* The sending: the MQTT packets and their socket calls, the
             * NINA's usrsock binding and event thread, its sockets,
             * protocol and SPI (NuttX's SPI driver and the board's chip
             * select).
             */

            *libapps.a:imu_raw_mqtt.c.*.o
            *libapps.a:imu_raw_packet.c.*.o
            *libapps.a:mqtt_min.c.*.o
            *libapps.a:mqtt_min_sock.c.*.o
            *libexternal.a:nina_usrsock.o
            *libexternal.a:nina_poller.o
            *libexternal.a:nina_sock.o
            *libexternal.a:nina_proto.o
            *libexternal.a:nina_wait.o
            *libexternal.a:nina_os.o
            *libexternal.a:nina_bus_rp2040.o
            *libarch.a:rp2040_spi.o
            *libboard.a:rp2040_spi.o

            /* RAM code end */
        ) *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
    } > flash

    .init_section : {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
    } > flash

    .ARM.extab : {
        *(.ARM.extab*)
    } > flash

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx : {
        *(.ARM.exidx*)
    } > flash
    __exidx_end = ABSOLUTE(.);

    _eronly = ABSOLUTE(.);

    .ram_vectors (COPY) : {
        *(.ram_vectors)
    } > sram

    .data : {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        *(.ram_code.*)

        /* The code that .text left out (the RAM code list above). */

        *(.text .text.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram AT > flash

    .noinit (NOLOAD) : {
        . = ALIGN(4);
        _snoinit = ABSOLUTE(.);
        *(.noinit .noinit.*)
        . = ALIGN(4);
        _enoinit = ABSOLUTE(.);
    } > sram AT > sram

    .flash_section : {
        . = ALIGN(4*1024);
        *(.flash.*)
    } > flash

    .bss : {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > sram

    /* Stabs debugging sections. */
    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }
}
