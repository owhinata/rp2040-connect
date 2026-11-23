# SPDX-License-Identifier: MIT
# The code that runs from RAM instead of the flash.  include() this file from
# the application's CMakeLists.txt after find_package(Zephyr).
#
# The RP2040 executes the flash through a 16 KiB cache (XIP).  The code on
# the 6667 Hz path of the sensor (the kernel objects, the work queue and
# RTIO, I2C0 and its DMA, the LSM6DSOX's FIFO stream, the IMU service) and on
# the send path (MQTT, the sockets, the NINA's sockets, protocol and SPI)
# does not fit in it together.  On the FreeRTOS build, the same placement
# took cpu_active while sending from 33.1% to 19.7% and the cache's misses
# from about 342,000 to 5,900 a second (freertos/app/usb-console/
# ram_code.cmake).  Only the code moves: the read-only data stays in the
# flash (moving it made no difference there).
#
# To check a placement on the board, read the XIP cache's counters while
# sending: clear them with "devmem 0x1400000c 32 0" and
# "devmem 0x14000010 32 0" (they saturate), then read them about 15 s later
# (the hits and the accesses).
#
# How: Zephyr's code relocation (CONFIG_CODE_DATA_RELOCATION in prj.conf,
# zephyr_code_relocate()) with the location RAM_TEXT.  Its generator
# (scripts/build/gen_relocate_app.py) lists the objects' .text sections in
# an output section .ram_text_reloc that comes before the flash's .text in
# the linker script, in RAM and loaded from the flash; arch_data_copy()
# copies it with .data in z_prep_c(), before the kernel starts.  NOKEEP:
# the functions that nothing calls are still left out of the image.
#
# The list names the sources; the generator finds their objects in the
# build directory (patch 0008, manifest/zephyr/patches, for objects named
# after a path relative to a module's directory).  A listed source that
# does not exist stops the configuration; after the link, a check
# (ram_code_check.py, on compile_commands.json, the objects and the map)
# stops the build when a listed object's code is not in RAM (a source no
# longer built, or not found), when code of an object not listed is in RAM,
# or when less than RAM_CODE_MIN_FREE bytes of RAM are left.

set(RAM_CODE_SOURCES
  # The kernel objects under the drivers and the application (what the
  # other builds' compatibility layer implements): k_sem, k_mutex, k_poll,
  # k_work, the timeouts, k_queue and k_mem_slab (RTIO's work items), the
  # atomic operations (functions on the Cortex-M0+), the memory blocks of
  # the FIFO's buffers with their bit array, RTIO's executor and work
  # queue, the sensor clock, gpio0 (the LSM6DSOX's INT1, the NINA's READY)
  # with hal_rpi_pico's GPIO.
  ${ZEPHYR_BASE}/kernel/sem.c
  ${ZEPHYR_BASE}/kernel/mutex.c
  ${ZEPHYR_BASE}/kernel/poll.c
  ${ZEPHYR_BASE}/kernel/work.c
  ${ZEPHYR_BASE}/kernel/timeout.c
  ${ZEPHYR_BASE}/kernel/queue.c
  ${ZEPHYR_BASE}/kernel/mem_slab.c
  ${ZEPHYR_BASE}/lib/os/atomic_c.c
  ${ZEPHYR_BASE}/lib/mem_blocks/mem_blocks.c
  ${ZEPHYR_BASE}/lib/utils/bitarray.c
  ${ZEPHYR_BASE}/subsys/rtio/rtio_executor.c
  ${ZEPHYR_BASE}/subsys/rtio/rtio_workq.c
  ${ZEPHYR_BASE}/drivers/sensor/sensor_clock_sys.c
  ${ZEPHYR_BASE}/drivers/gpio/gpio_rpi_pico.c
  ${ZEPHYR_HAL_RPI_PICO_MODULE_DIR}/src/rp2_common/hardware_gpio/gpio.c

  # I2C0 with its DMA reads (patch 0003), and the DMA controller's driver
  # (also the NINA's SPI).
  ${ZEPHYR_BASE}/drivers/i2c/i2c_dw.c
  ${ZEPHYR_BASE}/drivers/dma/dma_rpi_pico.c

  # The LSM6DSOX: the FIFO stream (status, read, word parse, gyro bias), its
  # RTIO, work item and INT1, and the stmemsc glue.  Not ST's register
  # driver (lsm6dso_reg.c), which only configures: the FIFO's reads go to
  # the bus directly.  Not the decoder (the sensor shell's).
  ${COMMON_DIR}/drivers/lsm6dso/lsm6dso.c
  ${COMMON_DIR}/drivers/lsm6dso/lsm6dso_stream.c
  ${COMMON_DIR}/drivers/lsm6dso/lsm6dso_fifo.c
  ${COMMON_DIR}/drivers/lsm6dso/lsm6dso_cal.c
  ${COMMON_DIR}/drivers/lsm6dso/port/zephyr/lsm6dso_rtio_stream.c
  ${COMMON_DIR}/drivers/lsm6dso/port/zephyr/lsm6dso_rtio.c
  ${COMMON_DIR}/drivers/lsm6dso/port/zephyr/lsm6dso_os.c
  ${COMMON_DIR}/drivers/lsm6dso/port/zephyr/lsm6dso_sensor.c
  ${ZEPHYR_BASE}/drivers/sensor/st/stmemsc/stmemsc_i2c.c

  # The IMU service's side of the sensor: the stream thread, the batches
  # and their queue, the time stamps.  Not time_sync.c (mostly the NTP
  # exchange and its thread) nor imu_cal.c (read at the start).
  ${COMMON_DIR}/imu/imu_stream.c
  ${COMMON_DIR}/imu/imu_raw_batch.c
  ${COMMON_DIR}/imu/imu_raw_queue.c
  ${COMMON_DIR}/imu/imu_raw_time.c
  ${COMMON_DIR}/imu/imu_tick_clock.c
  ${COMMON_DIR}/imu/time_offset.c

  # The sending: the MQTT packets and Zephyr's MQTT library, the sockets
  # (the offload's dispatch, the file descriptors, poll()), the NINA's
  # sockets, protocol, waits and Zephyr binding (socket offload, kernel
  # services, SPI1 and its lines), the PL022 SPI driver.
  ${COMMON_DIR}/imu/imu_raw_mqtt.c
  ${COMMON_DIR}/imu/imu_raw_packet.c
  ${ZEPHYR_BASE}/subsys/net/lib/mqtt/mqtt.c
  ${ZEPHYR_BASE}/subsys/net/lib/mqtt/mqtt_encoder.c
  ${ZEPHYR_BASE}/subsys/net/lib/mqtt/mqtt_decoder.c
  ${ZEPHYR_BASE}/subsys/net/lib/mqtt/mqtt_rx.c
  ${ZEPHYR_BASE}/subsys/net/lib/mqtt/mqtt_transport.c
  ${ZEPHYR_BASE}/subsys/net/lib/mqtt/mqtt_transport_socket_tcp.c
  ${ZEPHYR_BASE}/subsys/net/lib/sockets/sockets.c
  ${ZEPHYR_BASE}/subsys/net/lib/sockets/socket_offload.c
  ${ZEPHYR_BASE}/lib/os/zvfs/zvfs_fdtable.c
  ${ZEPHYR_BASE}/lib/os/zvfs/zvfs_poll.c
  ${COMMON_DIR}/drivers/nina/nina_sock.c
  ${COMMON_DIR}/drivers/nina/internal/nina_proto.c
  ${COMMON_DIR}/drivers/nina/internal/nina_wait.c
  ${COMMON_DIR}/drivers/nina/port/zephyr/nina_socket.c
  ${COMMON_DIR}/drivers/nina/port/zephyr/nina_os.c
  ${COMMON_DIR}/drivers/nina/port/zephyr/nina_bus.c
  ${ZEPHYR_BASE}/drivers/spi/spi_pl022.c
)

# What the check after the link requires to be left of the RAM, after the
# end of the image's RAM (_image_ram_end).  That rest is malloc()'s arena,
# and its one user is the NINA passthrough mode, which takes its 16 KiB
# buffer there (in the mode only; the stacks and the other buffers are
# static).  16 KiB and 2 KiB for malloc()'s own use are kept, as in the
# FreeRTOS build.
set(RAM_CODE_MIN_FREE 18432)

foreach(source IN LISTS RAM_CODE_SOURCES)
  if(NOT EXISTS ${source})
    message(FATAL_ERROR "ram_code.cmake: ${source} does not exist")
  endif()
endforeach()
if(NOT CONFIG_CODE_DATA_RELOCATION)
  message(FATAL_ERROR "ram_code.cmake needs CONFIG_CODE_DATA_RELOCATION (prj.conf)")
endif()

zephyr_code_relocate(FILES ${RAM_CODE_SOURCES} LOCATION RAM_TEXT NOKEEP)

set(ram_code_list ${CMAKE_CURRENT_BINARY_DIR}/ram_code_sources.txt)
list(JOIN RAM_CODE_SOURCES "\n" ram_code_text)
file(CONFIGURE OUTPUT ${ram_code_list} CONTENT "${ram_code_text}\n" @ONLY)
set(ram_code_check ${CMAKE_CURRENT_LIST_DIR}/ram_code_check.py)
set(ram_code_stamp ${CMAKE_CURRENT_BINARY_DIR}/ram_code_check.stamp)
add_custom_command(
  OUTPUT ${ram_code_stamp}
  COMMAND ${PYTHON_EXECUTABLE} ${ram_code_check}
          --build-dir ${APPLICATION_BINARY_DIR}
          --elf $<TARGET_FILE:${logical_target_for_zephyr_elf}>
          --sources ${ram_code_list}
          --min-free ${RAM_CODE_MIN_FREE}
  COMMAND ${CMAKE_COMMAND} -E touch ${ram_code_stamp}
  DEPENDS ${logical_target_for_zephyr_elf} ${ram_code_check} ${ram_code_list}
  COMMENT "Checking the code in RAM (ram_code.cmake)"
  VERBATIM
)
add_custom_target(ram_code_check ALL DEPENDS ${ram_code_stamp})
