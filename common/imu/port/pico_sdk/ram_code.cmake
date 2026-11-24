# SPDX-License-Identifier: MIT
# The code that runs from RAM instead of the flash, for the builds on the
# Pico SDK.  usb-console.cmake include()s this file and calls
#
#   pico_sdk_ram_code(<target>)
#
# after the drivers' functions have added their sources.  The OS adds the
# objects of its own part on those paths to RAM_CODE_OS_SOURCES before
# (FreeRTOS: the NINA core's binding, common/drivers/nina/port/freertos/nina_os.c).
#
# The RP2040 executes the flash through a 16 KiB cache (XIP).  The code on
# the 6667 Hz path of the sensor (the layer's kernel, work queue and RTIO,
# I2C0 and its DMA, the LSM6DSOX's FIFO stream, the IMU service) and on the
# send path (MQTT, the layer's sockets, the NINA's sockets, protocol and
# SPI) does not fit in it together: while sending, the cache missed about
# 342,000 times a second and the misses cost about 14 points of cpu_active
# (the FreeRTOS build).  With the code of the objects below in RAM,
# cpu_active while sending was
# 19.7% instead of 33.1% and the misses about 5,900 a second (measured on
# 2026-11-23).  Their read-only data stays in the flash: moving it made no
# difference.  The cost is about 49 KiB of RAM (the code and the
# long-branch stubs between it and the flash), taken from what newlib's
# malloc() can use (the shell's commands, the NINA passthrough mode's buffer):
# about 89 KiB is left.
#
# To check a placement on the board, read the XIP cache's counters while
# sending: clear them with "devmem 0x1400000c 32 0" and
# "devmem 0x14000010 32 0" (they saturate), then read them about 15 s later
# (the hits and the accesses).
#
# How: the Pico SDK's default_text_excludes.incl
# (pico-sdk/src/rp2_common/pico_standard_link/script_include) is replaced,
# through pico_add_linker_script_override_path(), by a copy generated in the
# build directory that adds the objects to the SDK's own EXCLUDE_FILE list
# (libgcc.a and libm.a).  The flash's .text then leaves out their .text*,
# and the SDK's .data, which crt0.S copies to RAM before anything else
# runs, takes it with its "*(.text*)", as it takes libgcc's.  The rest of
# the linker script is the SDK's.
#
# The lists name sources relative to the root of the repository, the Pico
# SDK's under PICO_SDK_PATH (the build's own tree of it): the
# drivers' cmake files list them too, but mixed with the code that does not
# need to be in RAM (the start, the shell's commands), and some are the
# SDK's.  A listed source that does not exist stops the configuration; after
# the link, a check reads the map and stops the build when a listed object's
# code is not in RAM (a source no longer built, or built under another name)
# or when less than RAM_CODE_MIN_FREE bytes are left above the heap's
# 2 KiB minimum.

set(RAM_CODE_SOURCES
  # The Zephyr compatibility layer under the drivers and the application:
  # k_sem, k_poll, the timeouts and k_work, k_mutex, RTIO and its work
  # queue, the memory blocks of the FIFO's buffers, the clock, gpio0 (the
  # LSM6DSOX's INT1) with the SDK's GPIO.
  common/compat/kernel_generic/zephyr_kernel.c
  common/compat/kernel_generic/zephyr_mutex.c
  common/compat/zephyr_work.c
  common/compat/zephyr_rtio.c
  common/compat/zephyr_rtio_workq.c
  common/compat/zephyr_mem_blocks.c
  common/compat/zephyr_sensor_clock.c
  common/compat/zephyr_gpio.c
  common/compat/port/pico_sdk/zephyr_sys_clock.c
  common/compat/port/pico_sdk/gpio_os_rp2040.c
  ${PICO_SDK_PATH}/src/rp2_common/hardware_gpio/gpio.c

  # I2C0 with its DMA reads (the FIFO's status and words).  I2C1, when it
  # is built, is the same object.
  common/drivers/i2c_dw/i2c_dw.c
  common/drivers/i2c_dw/port/pico_sdk/i2c_dw_dma_os_rp2040.c
  ${PICO_SDK_PATH}/src/rp2_common/hardware_dma/dma.c

  # The LSM6DSOX: the FIFO stream (status, read, word parse, gyro bias), its
  # RTIO and work item.  Not the ST register driver (lsm6dso_reg.c), which
  # only configures: the FIFO's reads go to the bus directly.
  common/drivers/lsm6dso/lsm6dso.c
  common/drivers/lsm6dso/lsm6dso_stream.c
  common/drivers/lsm6dso/lsm6dso_fifo.c
  common/drivers/lsm6dso/lsm6dso_cal.c
  common/drivers/lsm6dso/port/zephyr/lsm6dso_rtio_stream.c
  common/drivers/lsm6dso/port/zephyr/lsm6dso_rtio.c
  common/drivers/lsm6dso/port/zephyr/lsm6dso_os.c
  common/drivers/lsm6dso/port/zephyr/lsm6dso_sensor.c
  common/drivers/stmemsc/stmemsc_i2c.c
  common/drivers/lsm6dso/port/pico_sdk/lsm6dso_board.c

  # The IMU service's side of the sensor: the stream thread, the batches
  # and their queue, the time stamps.  Not time_sync.c (mostly the NTP
  # exchange and its thread) nor imu_cal.c (read at the start).
  common/imu/imu_stream.c
  common/imu/imu_raw_batch.c
  common/imu/imu_raw_queue.c
  common/imu/imu_raw_time.c
  common/imu/imu_tick_clock.c
  common/imu/time_offset.c

  # The sending: the MQTT packets, the layer's sockets, the NINA's sockets,
  # protocol and SPI (with its DMA and READY waits).
  common/imu/imu_raw_mqtt.c
  common/imu/imu_raw_packet.c
  common/imu/mqtt_min.c
  common/imu/mqtt_min_sock.c
  common/compat/net_offload/subsys/net/lib/sockets/sockets.c
  common/compat/net_offload/subsys/net/lib/sockets/socket_offload.c
  common/compat/net_offload/lib/os/zvfs/zvfs_fdtable.c
  common/compat/net_offload/lib/os/zvfs/zvfs_poll.c
  common/drivers/nina/nina_sock.c
  common/drivers/nina/internal/nina_proto.c
  common/drivers/nina/internal/nina_wait.c
  common/drivers/nina/port/zephyr/nina_socket.c
  common/drivers/nina/port/pico_sdk/nina_bus_rp2040.c
  common/drivers/nina/port/pico_sdk/nina_bus_dma_rp2040.c
  ${PICO_SDK_PATH}/src/rp2_common/hardware_spi/spi.c
)

# What the check after the link requires to be left for malloc() (above the
# heap's 2 KiB minimum).  A shell command takes about 1.2 KiB while it runs,
# plus a line longer than 127 characters while it is printed.  The NINA
# passthrough mode's buffer (16384 bytes) is the first allocation of that
# mode: newlib takes 16408 bytes from sbrk for it, less than the 18432 of
# the arena this leaves.
set(RAM_CODE_MIN_FREE 16384)

set(RAM_CODE_FILE ${CMAKE_CURRENT_LIST_FILE})

function(pico_sdk_ram_code target)
  get_filename_component(root ${RAM_CODE_FILE}/../../../../.. ABSOLUTE)
  set(objects)
  foreach(entry IN LISTS RAM_CODE_SOURCES RAM_CODE_OS_SOURCES)
    if(IS_ABSOLUTE ${entry})
      set(source ${entry})
    else()
      set(source ${root}/${entry})
    endif()
    if(NOT EXISTS ${source})
      message(FATAL_ERROR "pico_sdk_ram_code: ${entry} does not exist (${RAM_CODE_FILE})")
    endif()
    # CMake names an object after its source: relative to the source
    # directory when the source is under it, else the absolute path.
    file(RELATIVE_PATH relative ${CMAKE_CURRENT_SOURCE_DIR} ${source})
    if(relative MATCHES "^\\.\\./")
      list(APPEND objects "${target}.dir${source}${CMAKE_C_OUTPUT_EXTENSION}")
    else()
      list(APPEND objects "${target}.dir/${relative}${CMAKE_C_OUTPUT_EXTENSION}")
    endif()
  endforeach()

  set(dir ${CMAKE_CURRENT_BINARY_DIR}/ram_code)
  list(JOIN objects "\n    */" excluded)
  file(CONFIGURE OUTPUT ${dir}/default_text_excludes.incl
    CONTENT "/* Generated by common/imu/port/pico_sdk/ram_code.cmake */\n*(EXCLUDE_FILE(*libgcc.a: *libm.a:\n    */${excluded}) .text*)\n" @ONLY)
  list(JOIN objects "\n" listed)
  file(CONFIGURE OUTPUT ${dir}/objects.txt CONTENT "${listed}\n" @ONLY)
  pico_add_linker_script_override_path(${target} ${dir}
    FILES default_text_excludes.incl
  )

  add_custom_command(TARGET ${target} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -DRAM_CODE_OBJECTS=${dir}/objects.txt
            -DRAM_CODE_MAP=$<TARGET_FILE:${target}>.map -P ${RAM_CODE_FILE}
    VERBATIM
  )
endfunction()

# The check after the link (cmake -P this file): every listed object's code
# is in RAM, and RAM_CODE_MIN_FREE is left.
if(CMAKE_SCRIPT_MODE_FILE)
  cmake_minimum_required(VERSION 3.13)
  file(STRINGS ${RAM_CODE_OBJECTS} objects)
  # Input sections: " .name 0xaddress 0xsize object", or the name alone on
  # its line and the rest on the next.  The discarded ones come before
  # "Linker script and memory map".
  file(STRINGS ${RAM_CODE_MAP} lines
    REGEX "^Linker script|^ \\.|^ +0x[0-9a-f]+ +0x[0-9a-f]+ +[^ ]|^\\.heap |__HeapLimit = ")
  set(mapped FALSE)
  set(in_ram)
  set(in_flash)
  foreach(line IN LISTS lines)
    if(line MATCHES "^Linker script")
      set(mapped TRUE)
    elseif(NOT mapped)
    elseif(line MATCHES "^\\.heap +0x([0-9a-f]+) +0x([0-9a-f]+)")
      math(EXPR heap_end "0x${CMAKE_MATCH_1} + 0x${CMAKE_MATCH_2}")
    elseif(line MATCHES "^ +0x([0-9a-f]+) +__HeapLimit = ")
      math(EXPR heap_limit "0x${CMAKE_MATCH_1}")
    elseif(line MATCHES "^ (\\.[^ ]+)$")
      set(section ${CMAKE_MATCH_1})
    elseif(line MATCHES "^ (\\.[^ ]+)? +0x([0-9a-f]+) +0x([0-9a-f]+) +CMakeFiles/(.+)$")
      if(NOT CMAKE_MATCH_1 STREQUAL "")
        set(section ${CMAKE_MATCH_1})
      endif()
      set(address ${CMAKE_MATCH_2})
      set(size ${CMAKE_MATCH_3})
      set(object ${CMAKE_MATCH_4})
      if(section MATCHES "^\\.text" AND NOT size STREQUAL "0" AND object IN_LIST objects)
        if(address MATCHES "^2")
          list(APPEND in_ram ${object})
        else()
          list(APPEND in_flash ${object})
        endif()
      endif()
    endif()
  endforeach()

  set(errors)
  foreach(object IN LISTS objects)
    if(object IN_LIST in_flash OR NOT object IN_LIST in_ram)
      list(APPEND errors "  ${object}: its code is not in RAM")
    endif()
  endforeach()
  if(NOT DEFINED heap_end OR NOT DEFINED heap_limit)
    list(APPEND errors "  no .heap or __HeapLimit in ${RAM_CODE_MAP}")
  else()
    math(EXPR free "${heap_limit} - ${heap_end}")
    if(free LESS RAM_CODE_MIN_FREE)
      list(APPEND errors
        "  ${free} bytes are left for malloc(), fewer than RAM_CODE_MIN_FREE (${RAM_CODE_MIN_FREE})")
    endif()
  endif()
  if(errors)
    list(JOIN errors "\n" text)
    message(FATAL_ERROR "${RAM_CODE_FILE}:\n${text}")
  endif()
endif()
