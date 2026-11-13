/****************************************************************************
 * boards/arduino-nano-rp2040-connect/scripts/nano-flash.ld
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

    .text : {
        _stext = ABSOLUTE(.);
        *(.vectors)
        *(.text .text.*)
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
